#include "ToolManager.hpp"
#include "ToolCalibration.hpp"
#include "../kinematics/KinematicsService.hpp"
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <filesystem>

namespace fs = std::filesystem;

namespace robot_controller {
namespace tool {

ToolManager::ToolManager(kinematics::KinematicsService* kinematics)
    : m_kinematics(kinematics)
{
    // Create default "No Tool" entry
    ToolData defaultTool;
    defaultTool.id = "tool_default";
    defaultTool.name = "No Tool";
    defaultTool.description = "Default tool with no offset";
    defaultTool.isActive = true;
    m_tools[defaultTool.id] = defaultTool;
    m_activeToolId = defaultTool.id;

    spdlog::info("[ToolManager] Initialized with default tool");
}

// ============================================================================
// CRUD Operations
// ============================================================================

bool ToolManager::createTool(const ToolData& tool) {
    {
        std::lock_guard<std::mutex> lock(m_mutex);

        if (tool.id.empty()) {
            spdlog::error("[ToolManager] Cannot create tool with empty ID");
            return false;
        }

        if (m_tools.find(tool.id) != m_tools.end()) {
            spdlog::error("[ToolManager] Tool already exists: {}", tool.id);
            return false;
        }

        m_tools[tool.id] = tool;
        spdlog::info("[ToolManager] Created tool: {} ({})", tool.name, tool.id);
    }
    // autoSave calls saveToDirectory which also locks m_mutex,
    // so must be called OUTSIDE the lock scope to avoid deadlock
    autoSave();
    return true;
}

bool ToolManager::updateTool(const std::string& id, const ToolData& tool) {
    bool wasActive = false;
    {
        std::lock_guard<std::mutex> lock(m_mutex);

        auto it = m_tools.find(id);
        if (it == m_tools.end()) {
            spdlog::error("[ToolManager] Tool not found: {}", id);
            return false;
        }

        wasActive = it->second.isActive;
        it->second = tool;
        it->second.id = id; // Ensure ID doesn't change
        it->second.isActive = wasActive;

        spdlog::info("[ToolManager] Updated tool: {}", id);
    }
    // Notify OUTSIDE lock to avoid deadlock
    if (wasActive) {
        notifyToolChanged(id);
    }
    autoSave();

    return true;
}

bool ToolManager::deleteTool(const std::string& id) {
    bool wasActive = false;
    {
        std::lock_guard<std::mutex> lock(m_mutex);

        if (id == "tool_default") {
            spdlog::error("[ToolManager] Cannot delete default tool");
            return false;
        }

        auto it = m_tools.find(id);
        if (it == m_tools.end()) {
            spdlog::error("[ToolManager] Tool not found: {}", id);
            return false;
        }

        wasActive = (id == m_activeToolId);
        m_tools.erase(it);

        if (wasActive) {
            m_activeToolId = "tool_default";
            m_tools[m_activeToolId].isActive = true;
        }

        spdlog::info("[ToolManager] Deleted tool: {}", id);
    }
    // Notify OUTSIDE lock to avoid deadlock
    if (wasActive) {
        notifyToolChanged("tool_default");
    }
    autoSave();
    return true;
}

std::optional<ToolData> ToolManager::getTool(const std::string& id) const {
    std::lock_guard<std::mutex> lock(m_mutex);

    auto it = m_tools.find(id);
    if (it != m_tools.end()) {
        return it->second;
    }
    return std::nullopt;
}

std::vector<ToolData> ToolManager::getAllTools() const {
    std::lock_guard<std::mutex> lock(m_mutex);

    std::vector<ToolData> result;
    result.reserve(m_tools.size());
    for (const auto& [id, tool] : m_tools) {
        result.push_back(tool);
    }
    return result;
}

bool ToolManager::toolExists(const std::string& id) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_tools.find(id) != m_tools.end();
}

// ============================================================================
// Active Tool Management
// ============================================================================

bool ToolManager::setActiveTool(const std::string& id) {
    {
        std::lock_guard<std::mutex> lock(m_mutex);

        auto it = m_tools.find(id);
        if (it == m_tools.end()) {
            spdlog::error("[ToolManager] Cannot set active tool, not found: {}", id);
            return false;
        }

        // Deactivate current
        if (!m_activeToolId.empty() && m_tools.find(m_activeToolId) != m_tools.end()) {
            m_tools[m_activeToolId].isActive = false;
        }

        // Activate new
        m_activeToolId = id;
        it->second.isActive = true;

        spdlog::info("[ToolManager] Active tool set to: {} ({})", it->second.name, id);

        // Update kinematics with new tool offset
        if (m_kinematics) {
            m_kinematics->setToolOffset(it->second.tcp.toArray());
        }
    }
    // Notify OUTSIDE lock to avoid deadlock (callback may call getActiveTool)
    notifyToolChanged(id);
    autoSave();
    return true;
}

std::optional<ToolData> ToolManager::getActiveTool() const {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_activeToolId.empty()) {
        return std::nullopt;
    }

    auto it = m_tools.find(m_activeToolId);
    if (it != m_tools.end()) {
        return it->second;
    }
    return std::nullopt;
}

std::string ToolManager::getActiveToolId() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_activeToolId;
}

void ToolManager::setToolChangedCallback(ToolChangedCallback callback) {
    m_toolChangedCallback = std::move(callback);
}

// ============================================================================
// Calibration
// ============================================================================

bool ToolManager::startCalibration(CalibrationMethod method) {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_calibStatus.state == CalibrationState::COLLECTING_POINTS) {
        spdlog::warn("[ToolManager] Calibration already in progress");
        return false;
    }

    m_calibStatus.method = method;
    m_calibStatus.state = CalibrationState::COLLECTING_POINTS;
    m_calibStatus.pointsRequired = getRequiredPoints(method);
    m_calibStatus.pointsRecorded = 0;
    m_calibStatus.errorMessage.clear();
    m_calibStatus.result.reset();
    m_calibPoints.clear();
    m_calibFlangeFrames.clear();

    spdlog::info("[ToolManager] Started {} calibration, {} points required",
        calibrationMethodToString(method), m_calibStatus.pointsRequired);

    return true;
}

bool ToolManager::recordCalibrationPoint(const std::array<double, 6>& jointAngles) {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_calibStatus.state != CalibrationState::COLLECTING_POINTS) {
        spdlog::error("[ToolManager] Not in calibration mode");
        return false;
    }

    if (m_calibStatus.pointsRecorded >= m_calibStatus.pointsRequired) {
        spdlog::warn("[ToolManager] All calibration points already recorded");
        return false;
    }

    m_calibPoints.push_back(jointAngles);

    // Calculate flange frame from FK
    if (m_kinematics) {
        Eigen::Matrix4d flangeFrame = m_kinematics->calculateFK(jointAngles);
        m_calibFlangeFrames.push_back(flangeFrame);
    }

    m_calibStatus.pointsRecorded++;
    spdlog::info("[ToolManager] Recorded calibration point {}/{}",
        m_calibStatus.pointsRecorded, m_calibStatus.pointsRequired);

    return true;
}

std::optional<ToolTCP> ToolManager::calculateTCP() {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_calibStatus.pointsRecorded < m_calibStatus.pointsRequired) {
        m_calibStatus.state = CalibrationState::FAILED;
        m_calibStatus.errorMessage = "Not enough calibration points";
        spdlog::error("[ToolManager] {}", m_calibStatus.errorMessage);
        return std::nullopt;
    }

    m_calibStatus.state = CalibrationState::CALCULATING;

    std::optional<ToolTCP> result;

    switch (m_calibStatus.method) {
        case CalibrationMethod::FOUR_POINT:
            result = ToolCalibration::calculate4Point(m_calibFlangeFrames);
            break;
        case CalibrationMethod::SIX_POINT:
            result = ToolCalibration::calculate6Point(m_calibFlangeFrames);
            break;
        default:
            m_calibStatus.state = CalibrationState::FAILED;
            m_calibStatus.errorMessage = "Invalid calibration method";
            return std::nullopt;
    }

    if (result) {
        m_calibStatus.state = CalibrationState::COMPLETED;
        m_calibStatus.result = result;
        spdlog::info("[ToolManager] TCP calculated: X={:.2f}, Y={:.2f}, Z={:.2f}",
            result->x, result->y, result->z);
    } else {
        m_calibStatus.state = CalibrationState::FAILED;
        m_calibStatus.errorMessage = "Calibration calculation failed";
    }

    return result;
}

CalibrationStatus ToolManager::getCalibrationStatus() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_calibStatus;
}

void ToolManager::cancelCalibration() {
    std::lock_guard<std::mutex> lock(m_mutex);

    m_calibStatus.state = CalibrationState::IDLE;
    m_calibPoints.clear();
    m_calibFlangeFrames.clear();

    spdlog::info("[ToolManager] Calibration cancelled");
}

bool ToolManager::applyCalibrationToTool(const std::string& toolId, const ToolTCP& tcp) {
    std::lock_guard<std::mutex> lock(m_mutex);

    auto it = m_tools.find(toolId);
    if (it == m_tools.end()) {
        spdlog::error("[ToolManager] Tool not found: {}", toolId);
        return false;
    }

    it->second.tcp = tcp;
    spdlog::info("[ToolManager] Applied calibration to tool: {}", toolId);

    if (it->second.isActive && m_kinematics) {
        m_kinematics->setToolOffset(tcp.toArray());
    }

    return true;
}

// ============================================================================
// Persistence
// ============================================================================

bool ToolManager::saveToDirectory(const std::string& path) {
    std::lock_guard<std::mutex> lock(m_mutex);

    try {
        fs::create_directories(path);

        for (const auto& [id, tool] : m_tools) {
            std::string filepath = (fs::path(path) / (id + ".yaml")).string();

            YAML::Emitter out;
            out << YAML::BeginMap;
            out << YAML::Key << "tool" << YAML::BeginMap;
            out << YAML::Key << "id" << YAML::Value << tool.id;
            out << YAML::Key << "name" << YAML::Value << tool.name;
            out << YAML::Key << "description" << YAML::Value << tool.description;

            out << YAML::Key << "tcp" << YAML::BeginMap;
            out << YAML::Key << "x" << YAML::Value << tool.tcp.x;
            out << YAML::Key << "y" << YAML::Value << tool.tcp.y;
            out << YAML::Key << "z" << YAML::Value << tool.tcp.z;
            out << YAML::Key << "rx" << YAML::Value << tool.tcp.rx;
            out << YAML::Key << "ry" << YAML::Value << tool.tcp.ry;
            out << YAML::Key << "rz" << YAML::Value << tool.tcp.rz;
            out << YAML::EndMap;

            out << YAML::Key << "inertia" << YAML::BeginMap;
            out << YAML::Key << "mass" << YAML::Value << tool.inertia.mass;
            out << YAML::Key << "cog" << YAML::Flow << YAML::BeginSeq
                << tool.inertia.cogX << tool.inertia.cogY << tool.inertia.cogZ
                << YAML::EndSeq;
            out << YAML::EndMap;

            // Visual mesh (optional)
            if (!tool.visualMeshPath.empty()) {
                out << YAML::Key << "visual_mesh" << YAML::Value << tool.visualMeshPath;

                out << YAML::Key << "mesh_offset" << YAML::BeginMap;
                out << YAML::Key << "x" << YAML::Value << tool.meshOffsetX;
                out << YAML::Key << "y" << YAML::Value << tool.meshOffsetY;
                out << YAML::Key << "z" << YAML::Value << tool.meshOffsetZ;
                out << YAML::Key << "rx" << YAML::Value << tool.meshOffsetRx;
                out << YAML::Key << "ry" << YAML::Value << tool.meshOffsetRy;
                out << YAML::Key << "rz" << YAML::Value << tool.meshOffsetRz;
                out << YAML::EndMap;

                if (tool.meshScale != 1.0) {
                    out << YAML::Key << "mesh_scale" << YAML::Value << tool.meshScale;
                }
            }

            out << YAML::EndMap; // tool
            out << YAML::EndMap;

            std::ofstream fout(filepath);
            fout << out.c_str();
        }

        // Save active tool reference
        YAML::Emitter activeOut;
        activeOut << YAML::BeginMap;
        activeOut << YAML::Key << "active_tool" << YAML::Value << m_activeToolId;
        activeOut << YAML::EndMap;

        std::ofstream activeFile((fs::path(path) / "active.yaml").string());
        activeFile << activeOut.c_str();

        spdlog::info("[ToolManager] Saved {} tools to {}", m_tools.size(), path);
        return true;

    } catch (const std::exception& e) {
        spdlog::error("[ToolManager] Failed to save tools: {}", e.what());
        return false;
    }
}

bool ToolManager::loadFromDirectory(const std::string& path) {
    std::lock_guard<std::mutex> lock(m_mutex);

    try {
        if (!fs::exists(path)) {
            spdlog::warn("[ToolManager] Tool directory not found: {}", path);
            return false;
        }

        for (const auto& entry : fs::directory_iterator(path)) {
            if (entry.path().extension() == ".yaml" && entry.path().stem() != "active") {
                loadToolFromFile(entry.path().string());
            }
        }

        // Load active tool reference
        auto activePath = fs::path(path) / "active.yaml";
        if (fs::exists(activePath)) {
            YAML::Node activeNode = YAML::LoadFile(activePath.string());
            if (activeNode["active_tool"]) {
                std::string activeId = activeNode["active_tool"].as<std::string>();
                if (m_tools.find(activeId) != m_tools.end()) {
                    m_activeToolId = activeId;
                    m_tools[m_activeToolId].isActive = true;
                }
            }
        }

        // Remember config directory for auto-save
        m_configDir = path;

        spdlog::info("[ToolManager] Loaded {} tools from {}", m_tools.size(), path);
        return true;

    } catch (const std::exception& e) {
        spdlog::error("[ToolManager] Failed to load tools: {}", e.what());
        return false;
    }
}

bool ToolManager::saveToolToFile(const std::string& id, const std::string& filepath) {
    auto tool = getTool(id);
    if (!tool) {
        return false;
    }

    // Implementation similar to saveToDirectory but for single tool
    return true;
}

bool ToolManager::loadToolFromFile(const std::string& filepath) {
    try {
        YAML::Node node = YAML::LoadFile(filepath);

        if (!node["tool"]) {
            return false;
        }

        auto toolNode = node["tool"];
        ToolData tool;
        tool.id = toolNode["id"].as<std::string>();
        tool.name = toolNode["name"].as<std::string>();
        tool.description = toolNode["description"].as<std::string>("");

        if (toolNode["tcp"]) {
            auto tcpNode = toolNode["tcp"];
            tool.tcp.x = tcpNode["x"].as<double>(0.0);
            tool.tcp.y = tcpNode["y"].as<double>(0.0);
            tool.tcp.z = tcpNode["z"].as<double>(0.0);
            tool.tcp.rx = tcpNode["rx"].as<double>(0.0);
            tool.tcp.ry = tcpNode["ry"].as<double>(0.0);
            tool.tcp.rz = tcpNode["rz"].as<double>(0.0);
        }

        if (toolNode["inertia"]) {
            auto inertiaNode = toolNode["inertia"];
            tool.inertia.mass = inertiaNode["mass"].as<double>(0.0);
            if (inertiaNode["cog"] && inertiaNode["cog"].IsSequence()) {
                tool.inertia.cogX = inertiaNode["cog"][0].as<double>(0.0);
                tool.inertia.cogY = inertiaNode["cog"][1].as<double>(0.0);
                tool.inertia.cogZ = inertiaNode["cog"][2].as<double>(0.0);
            }
        }

        // Visual mesh (optional, backward compatible)
        if (toolNode["visual_mesh"]) {
            tool.visualMeshPath = toolNode["visual_mesh"].as<std::string>("");
        }
        if (toolNode["mesh_offset"]) {
            auto moNode = toolNode["mesh_offset"];
            tool.meshOffsetX = moNode["x"].as<double>(0.0);
            tool.meshOffsetY = moNode["y"].as<double>(0.0);
            tool.meshOffsetZ = moNode["z"].as<double>(0.0);
            tool.meshOffsetRx = moNode["rx"].as<double>(0.0);
            tool.meshOffsetRy = moNode["ry"].as<double>(0.0);
            tool.meshOffsetRz = moNode["rz"].as<double>(0.0);
        }
        if (toolNode["mesh_scale"]) {
            tool.meshScale = toolNode["mesh_scale"].as<double>(1.0);
        }

        m_tools[tool.id] = tool;
        spdlog::debug("[ToolManager] Loaded tool: {} from {}", tool.id, filepath);
        return true;

    } catch (const std::exception& e) {
        spdlog::error("[ToolManager] Failed to load tool from {}: {}", filepath, e.what());
        return false;
    }
}

// ============================================================================
// Private Methods
// ============================================================================

void ToolManager::notifyToolChanged(const std::string& toolId) {
    if (m_toolChangedCallback) {
        m_toolChangedCallback(toolId);
    }
}

int ToolManager::getRequiredPoints(CalibrationMethod method) const {
    switch (method) {
        case CalibrationMethod::FOUR_POINT: return 4;
        case CalibrationMethod::SIX_POINT: return 6;
        default: return 0;
    }
}

void ToolManager::autoSave() {
    if (m_configDir.empty()) return;

    // Run save without holding m_mutex (caller already released it)
    if (!saveToDirectory(m_configDir)) {
        spdlog::warn("[ToolManager] Auto-save failed to {}", m_configDir);
    } else {
        spdlog::debug("[ToolManager] Auto-saved to {}", m_configDir);
    }
}

} // namespace tool
} // namespace robot_controller
