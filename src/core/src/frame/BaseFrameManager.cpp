#include "BaseFrameManager.hpp"
#include "BaseCalibration.hpp"
#include "../logging/Logger.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <filesystem>

namespace robot_controller {
namespace frame {

namespace fs = std::filesystem;

BaseFrameManager::BaseFrameManager() {
    initializeDefaultBase();
    LOG_INFO("BaseFrameManager initialized");
}

void BaseFrameManager::initializeDefaultBase() {
    BaseFrame world;
    world.id = "world";
    world.name = "World Frame";
    world.description = "Robot base coordinate system";
    world.frame = Frame{0, 0, 0, 0, 0, 0};
    world.isActive = true;

    m_bases["world"] = world;
    m_activeBaseId = "world";
}

// ============================================================================
// CRUD Operations
// ============================================================================

bool BaseFrameManager::createBase(const BaseFrame& base) {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (base.id.empty()) {
        LOG_ERROR("Cannot create base with empty ID");
        return false;
    }

    if (m_bases.find(base.id) != m_bases.end()) {
        LOG_ERROR("Base frame already exists: {}", base.id);
        return false;
    }

    m_bases[base.id] = base;
    LOG_INFO("Created base frame: {}", base.id);

    return true;
}

bool BaseFrameManager::updateBase(const std::string& id, const BaseFrame& base) {
    std::lock_guard<std::mutex> lock(m_mutex);

    auto it = m_bases.find(id);
    if (it == m_bases.end()) {
        LOG_ERROR("Base frame not found: {}", id);
        return false;
    }

    // Preserve active state
    bool wasActive = it->second.isActive;
    it->second = base;
    it->second.id = id;  // Ensure ID doesn't change
    it->second.isActive = wasActive;

    LOG_INFO("Updated base frame: {}", id);

    if (m_baseChangedCallback && wasActive) {
        m_baseChangedCallback(id);
    }

    return true;
}

bool BaseFrameManager::deleteBase(const std::string& id) {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (id == "world") {
        LOG_ERROR("Cannot delete world frame");
        return false;
    }

    auto it = m_bases.find(id);
    if (it == m_bases.end()) {
        LOG_ERROR("Base frame not found: {}", id);
        return false;
    }

    bool wasActive = it->second.isActive;
    m_bases.erase(it);

    // If deleted the active base, switch to world
    if (wasActive) {
        m_activeBaseId = "world";
        m_bases["world"].isActive = true;
        if (m_baseChangedCallback) {
            m_baseChangedCallback("world");
        }
    }

    LOG_INFO("Deleted base frame: {}", id);
    return true;
}

std::optional<BaseFrame> BaseFrameManager::getBase(const std::string& id) const {
    std::lock_guard<std::mutex> lock(m_mutex);

    auto it = m_bases.find(id);
    if (it != m_bases.end()) {
        return it->second;
    }
    return std::nullopt;
}

std::vector<BaseFrame> BaseFrameManager::getAllBases() const {
    std::lock_guard<std::mutex> lock(m_mutex);

    std::vector<BaseFrame> result;
    result.reserve(m_bases.size());

    for (const auto& [id, base] : m_bases) {
        result.push_back(base);
    }

    return result;
}

// ============================================================================
// Active Base Management
// ============================================================================

bool BaseFrameManager::setActiveBase(const std::string& id) {
    std::lock_guard<std::mutex> lock(m_mutex);

    auto it = m_bases.find(id);
    if (it == m_bases.end()) {
        LOG_ERROR("Base frame not found: {}", id);
        return false;
    }

    // Deactivate current
    auto currentIt = m_bases.find(m_activeBaseId);
    if (currentIt != m_bases.end()) {
        currentIt->second.isActive = false;
    }

    // Activate new
    it->second.isActive = true;
    m_activeBaseId = id;

    LOG_INFO("Active base changed to: {}", id);

    if (m_baseChangedCallback) {
        m_baseChangedCallback(id);
    }

    return true;
}

std::optional<BaseFrame> BaseFrameManager::getActiveBase() const {
    return getBase(m_activeBaseId);
}

std::string BaseFrameManager::getActiveBaseId() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_activeBaseId;
}

// ============================================================================
// Coordinate Transformation
// ============================================================================

Frame BaseFrameManager::transformToBase(const Frame& worldFrame) const {
    Eigen::Matrix4d baseInverse = getActiveBaseInverseMatrix();
    Eigen::Matrix4d worldMat = worldFrame.toMatrix();
    Eigen::Matrix4d baseMat = baseInverse * worldMat;
    return Frame::fromMatrix(baseMat);
}

Frame BaseFrameManager::transformToWorld(const Frame& baseFrame) const {
    Eigen::Matrix4d baseMat = getActiveBaseMatrix();
    Eigen::Matrix4d frameMat = baseFrame.toMatrix();
    Eigen::Matrix4d worldMat = baseMat * frameMat;
    return Frame::fromMatrix(worldMat);
}

Eigen::Matrix4d BaseFrameManager::getActiveBaseMatrix() const {
    std::lock_guard<std::mutex> lock(m_mutex);

    auto it = m_bases.find(m_activeBaseId);
    if (it != m_bases.end()) {
        return it->second.frame.toMatrix();
    }

    return Eigen::Matrix4d::Identity();
}

Eigen::Matrix4d BaseFrameManager::getActiveBaseInverseMatrix() const {
    return getActiveBaseMatrix().inverse();
}

// ============================================================================
// Calibration
// ============================================================================

void BaseFrameManager::startCalibration(BaseCalibrationMethod method) {
    std::lock_guard<std::mutex> lock(m_mutex);

    m_calibStatus.state = BaseCalibrationState::IN_PROGRESS;
    m_calibStatus.method = method;
    m_calibStatus.pointsRequired = getRequiredPointCount(method);
    m_calibStatus.pointsRecorded = 0;
    m_calibStatus.errorMessage.clear();

    m_calibPoints.clear();
    m_calibPoints.resize(m_calibStatus.pointsRequired);

    for (int i = 0; i < m_calibStatus.pointsRequired; ++i) {
        m_calibPoints[i].pointIndex = i;
        m_calibPoints[i].pointName = getCalibrationPointName(i);
        m_calibPoints[i].recorded = false;
    }

    m_calibStatus.currentPointName = m_calibPoints[0].pointName;

    LOG_INFO("Started base calibration: method={}, points={}",
             baseCalibrationMethodToString(method),
             m_calibStatus.pointsRequired);
}

bool BaseFrameManager::recordCalibrationPoint(int pointIndex,
                                               const std::array<double, 6>& jointAngles,
                                               const std::array<double, 3>& tcpPosition) {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_calibStatus.state != BaseCalibrationState::IN_PROGRESS) {
        LOG_ERROR("No calibration in progress");
        return false;
    }

    if (pointIndex < 0 || pointIndex >= static_cast<int>(m_calibPoints.size())) {
        LOG_ERROR("Invalid point index: {}", pointIndex);
        return false;
    }

    m_calibPoints[pointIndex].jointAngles = jointAngles;
    m_calibPoints[pointIndex].tcpPosition = tcpPosition;
    m_calibPoints[pointIndex].recorded = true;

    // Count recorded points
    m_calibStatus.pointsRecorded = 0;
    for (const auto& pt : m_calibPoints) {
        if (pt.recorded) m_calibStatus.pointsRecorded++;
    }

    // Update current point name to next unrecorded
    for (const auto& pt : m_calibPoints) {
        if (!pt.recorded) {
            m_calibStatus.currentPointName = pt.pointName;
            break;
        }
    }

    LOG_INFO("Recorded calibration point {}: {} at ({:.1f}, {:.1f}, {:.1f})",
             pointIndex, m_calibPoints[pointIndex].pointName,
             tcpPosition[0], tcpPosition[1], tcpPosition[2]);

    return true;
}

std::optional<Frame> BaseFrameManager::calculateBaseFrame() {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_calibStatus.state != BaseCalibrationState::IN_PROGRESS) {
        LOG_ERROR("No calibration in progress");
        return std::nullopt;
    }

    // Validate all points recorded
    for (const auto& pt : m_calibPoints) {
        if (!pt.recorded) {
            m_calibStatus.errorMessage = "Not all points recorded";
            LOG_ERROR("Calibration incomplete: point {} not recorded", pt.pointName);
            return std::nullopt;
        }
    }

    // Validate points
    std::string validationError = BaseCalibration::validatePoints(m_calibPoints, m_calibStatus.method);
    if (!validationError.empty()) {
        m_calibStatus.state = BaseCalibrationState::ERROR;
        m_calibStatus.errorMessage = validationError;
        LOG_ERROR("Calibration validation failed: {}", validationError);
        return std::nullopt;
    }

    // Calculate frame
    auto result = BaseCalibration::calculateFromPoints(m_calibPoints, m_calibStatus.method);

    if (result) {
        m_calibStatus.state = BaseCalibrationState::COMPLETED;
        LOG_INFO("Base calibration completed: origin=({:.1f}, {:.1f}, {:.1f})",
                 result->x, result->y, result->z);
    } else {
        m_calibStatus.state = BaseCalibrationState::ERROR;
        m_calibStatus.errorMessage = "Calculation failed";
        LOG_ERROR("Base calibration calculation failed");
    }

    return result;
}

void BaseFrameManager::cancelCalibration() {
    std::lock_guard<std::mutex> lock(m_mutex);

    m_calibStatus.state = BaseCalibrationState::IDLE;
    m_calibPoints.clear();

    LOG_INFO("Base calibration cancelled");
}

BaseCalibrationStatus BaseFrameManager::getCalibrationStatus() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_calibStatus;
}

bool BaseFrameManager::isCalibrating() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_calibStatus.state == BaseCalibrationState::IN_PROGRESS;
}

int BaseFrameManager::getRequiredPointCount(BaseCalibrationMethod method) const {
    switch (method) {
        case BaseCalibrationMethod::DIRECT_INPUT: return 0;
        case BaseCalibrationMethod::THREE_POINT:  return 3;
        case BaseCalibrationMethod::FOUR_POINT:   return 4;
        default: return 3;
    }
}

// ============================================================================
// Persistence
// ============================================================================

bool BaseFrameManager::saveToDirectory(const std::string& directory) {
    std::lock_guard<std::mutex> lock(m_mutex);

    try {
        fs::create_directories(directory);

        for (const auto& [id, base] : m_bases) {
            YAML::Emitter out;
            out << YAML::BeginMap;
            out << YAML::Key << "base" << YAML::Value << YAML::BeginMap;
            out << YAML::Key << "id" << YAML::Value << base.id;
            out << YAML::Key << "name" << YAML::Value << base.name;
            out << YAML::Key << "description" << YAML::Value << base.description;
            out << YAML::Key << "frame" << YAML::Value << YAML::BeginMap;
            out << YAML::Key << "x" << YAML::Value << base.frame.x;
            out << YAML::Key << "y" << YAML::Value << base.frame.y;
            out << YAML::Key << "z" << YAML::Value << base.frame.z;
            out << YAML::Key << "rx" << YAML::Value << base.frame.rx;
            out << YAML::Key << "ry" << YAML::Value << base.frame.ry;
            out << YAML::Key << "rz" << YAML::Value << base.frame.rz;
            out << YAML::EndMap;
            out << YAML::EndMap;
            out << YAML::EndMap;

            std::string filePath = directory + "/" + id + ".yaml";
            std::ofstream fout(filePath);
            fout << out.c_str();
        }

        LOG_INFO("Saved {} base frames to {}", m_bases.size(), directory);
        return true;
    }
    catch (const std::exception& ex) {
        LOG_ERROR("Failed to save base frames: {}", ex.what());
        return false;
    }
}

bool BaseFrameManager::loadFromDirectory(const std::string& directory) {
    std::lock_guard<std::mutex> lock(m_mutex);

    try {
        if (!fs::exists(directory)) {
            LOG_WARN("Base frames directory not found: {}", directory);
            return false;
        }

        int count = 0;
        for (const auto& entry : fs::directory_iterator(directory)) {
            if (entry.path().extension() == ".yaml") {
                YAML::Node doc = YAML::LoadFile(entry.path().string());

                if (doc["base"]) {
                    auto baseNode = doc["base"];
                    BaseFrame base;
                    base.id = baseNode["id"].as<std::string>();
                    base.name = baseNode["name"].as<std::string>();
                    base.description = baseNode["description"].as<std::string>("");

                    if (baseNode["frame"]) {
                        auto f = baseNode["frame"];
                        base.frame.x = f["x"].as<double>(0.0);
                        base.frame.y = f["y"].as<double>(0.0);
                        base.frame.z = f["z"].as<double>(0.0);
                        base.frame.rx = f["rx"].as<double>(0.0);
                        base.frame.ry = f["ry"].as<double>(0.0);
                        base.frame.rz = f["rz"].as<double>(0.0);
                    }

                    m_bases[base.id] = base;
                    count++;
                }
            }
        }

        // Ensure world frame exists and is active
        if (m_bases.find("world") == m_bases.end()) {
            initializeDefaultBase();
        }
        m_bases[m_activeBaseId].isActive = true;

        LOG_INFO("Loaded {} base frames from {}", count, directory);
        return true;
    }
    catch (const std::exception& ex) {
        LOG_ERROR("Failed to load base frames: {}", ex.what());
        return false;
    }
}

void BaseFrameManager::setBaseChangedCallback(BaseChangedCallback callback) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_baseChangedCallback = std::move(callback);
}

} // namespace frame
} // namespace robot_controller
