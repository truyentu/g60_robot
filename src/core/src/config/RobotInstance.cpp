/**
 * @file RobotInstance.cpp
 * @brief Robot Instance Manager implementation
 */

#include "RobotInstance.hpp"
#include "../logging/Logger.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>

namespace robot_controller {
namespace config {

RobotInstanceManager::RobotInstanceManager(RobotCatalog& catalog)
    : m_catalog(catalog) {
}

bool RobotInstanceManager::loadInstances(const std::filesystem::path& instancesDir) {
    m_instancesDir = instancesDir;
    m_instances.clear();
    m_activeInstanceId.clear();

    if (!std::filesystem::exists(instancesDir)) {
        LOG_WARN("Instances directory does not exist: {}", instancesDir.string());
        // Create directory
        std::filesystem::create_directories(instancesDir);
        return true;  // Empty but valid
    }

    // Load each instance directory
    for (const auto& entry : std::filesystem::directory_iterator(instancesDir)) {
        if (entry.is_directory()) {
            if (loadInstance(entry.path())) {
                LOG_DEBUG("Loaded instance: {}", entry.path().filename().string());
            }
        }
    }

    // Find active instance
    for (const auto& [id, instance] : m_instances) {
        if (instance.active) {
            m_activeInstanceId = id;
            LOG_INFO("Active robot instance: {}", id);
            break;
        }
    }

    // If no active instance, use first one
    if (m_activeInstanceId.empty() && !m_instances.empty()) {
        m_activeInstanceId = m_instances.begin()->first;
        m_instances[m_activeInstanceId].active = true;
        LOG_INFO("Set default active instance: {}", m_activeInstanceId);
    }

    LOG_INFO("Robot instances loaded: {} instances", m_instances.size());
    return true;
}

bool RobotInstanceManager::loadInstance(const std::filesystem::path& instanceDir) {
    try {
        RobotInstance instance;
        instance.instanceId = instanceDir.filename().string();

        // Load instance.yaml
        auto instancePath = instanceDir / "instance.yaml";
        if (!loadInstanceConfig(instancePath, instance)) {
            return false;
        }

        // Load calibration.yaml (optional)
        auto calibPath = instanceDir / "calibration.yaml";
        if (std::filesystem::exists(calibPath)) {
            loadCalibration(calibPath, instance);
        }

        // Load base model and merge
        auto model = m_catalog.loadModel(instance.baseModelId);
        if (!model) {
            LOG_ERROR("Instance {} references unknown model: {}",
                     instance.instanceId, instance.baseModelId);
            return false;
        }

        // Merge model config with instance overrides
        instance.config = mergeModelAndInstance(*model, instance);

        m_instances[instance.instanceId] = instance;
        return true;

    } catch (const std::exception& e) {
        LOG_ERROR("Error loading instance from {}: {}", instanceDir.string(), e.what());
        return false;
    }
}

bool RobotInstanceManager::loadInstanceConfig(const std::filesystem::path& instancePath,
                                               RobotInstance& instance) {
    try {
        if (!std::filesystem::exists(instancePath)) {
            LOG_ERROR("Instance config not found: {}", instancePath.string());
            return false;
        }

        YAML::Node root = YAML::LoadFile(instancePath.string());

        instance.baseModelId = root["base_model"].as<std::string>();

        if (root["created"]) {
            instance.created = root["created"].as<std::string>();
        }
        if (root["last_modified"]) {
            instance.lastModified = root["last_modified"].as<std::string>();
        }
        if (root["active"]) {
            instance.active = root["active"].as<bool>();
        }

        // Load overrides
        if (root["overrides"]) {
            auto overrides = root["overrides"];

            // TCP override
            if (overrides["tcp_offset"]) {
                auto tcp = overrides["tcp_offset"];
                instance.tcpOverride.x = tcp["x"].as<double>(0.0);
                instance.tcpOverride.y = tcp["y"].as<double>(0.0);
                instance.tcpOverride.z = tcp["z"].as<double>(0.0);
                instance.tcpOverride.rx = tcp["rx"].as<double>(0.0);
                instance.tcpOverride.ry = tcp["ry"].as<double>(0.0);
                instance.tcpOverride.rz = tcp["rz"].as<double>(0.0);
                instance.hasTcpOverride = true;
            }
        }

        return true;

    } catch (const std::exception& e) {
        LOG_ERROR("Error loading instance config: {}", e.what());
        return false;
    }
}

bool RobotInstanceManager::loadCalibration(const std::filesystem::path& calibPath,
                                            RobotInstance& instance) {
    try {
        YAML::Node root = YAML::LoadFile(calibPath.string());

        if (root["calibration_date"]) {
            instance.calibration.calibrationDate = root["calibration_date"].as<std::string>();
        }
        if (root["calibrated_by"]) {
            instance.calibration.calibratedBy = root["calibrated_by"].as<std::string>();
        }
        if (root["notes"]) {
            instance.calibration.notes = root["notes"].as<std::string>();
        }

        // Load joint offsets
        if (root["joint_offsets"] && root["joint_offsets"].IsSequence()) {
            for (const auto& jo : root["joint_offsets"]) {
                int joint = jo["joint"].as<int>() - 1;  // Convert 1-based to 0-based
                if (joint >= 0 && joint < 6) {
                    instance.calibration.jointOffsets[joint] = jo["offset_deg"].as<double>(0.0);
                }
            }
        }

        return true;

    } catch (const std::exception& e) {
        LOG_WARN("Error loading calibration: {}", e.what());
        return false;
    }
}

RobotConfig RobotInstanceManager::mergeModelAndInstance(const RobotModel& model,
                                                         const RobotInstance& instance) const {
    RobotConfig config = model.toRobotConfig();

    // Apply TCP override if present
    if (instance.hasTcpOverride) {
        config.tcp_offset = instance.tcpOverride;
    }

    // Apply calibration offsets to DH parameters theta_offset
    // Note: In production, you might want to handle this differently
    // For now, we add the calibration offset to theta_offset
    if (instance.calibration.hasOffsets()) {
        for (size_t i = 0; i < 6 && i < config.dh_parameters.size(); ++i) {
            config.dh_parameters[i].theta_offset += instance.calibration.jointOffsets[i];
        }
        LOG_DEBUG("Applied calibration offsets to instance: {}", instance.instanceId);
    }

    return config;
}

std::optional<RobotInstance> RobotInstanceManager::getActiveInstance() const {
    if (m_activeInstanceId.empty()) {
        return std::nullopt;
    }

    auto it = m_instances.find(m_activeInstanceId);
    if (it != m_instances.end()) {
        return it->second;
    }
    return std::nullopt;
}

bool RobotInstanceManager::setActiveInstance(const std::string& instanceId) {
    auto it = m_instances.find(instanceId);
    if (it == m_instances.end()) {
        LOG_ERROR("Instance not found: {}", instanceId);
        return false;
    }

    // Deactivate previous
    if (!m_activeInstanceId.empty()) {
        m_instances[m_activeInstanceId].active = false;
    }

    // Activate new
    m_activeInstanceId = instanceId;
    m_instances[instanceId].active = true;

    LOG_INFO("Active instance changed to: {}", instanceId);
    notifyInstanceChanged(m_instances[instanceId]);

    return true;
}

bool RobotInstanceManager::changeInstanceModel(const std::string& instanceId,
                                                const std::string& modelId) {
    auto it = m_instances.find(instanceId);
    if (it == m_instances.end()) {
        LOG_ERROR("Instance not found: {}", instanceId);
        return false;
    }

    // Check model exists
    auto model = m_catalog.loadModel(modelId);
    if (!model) {
        LOG_ERROR("Model not found in catalog: {}", modelId);
        return false;
    }

    // Update instance
    it->second.baseModelId = modelId;
    it->second.config = mergeModelAndInstance(*model, it->second);

    LOG_INFO("Instance {} model changed to: {}", instanceId, modelId);

    // Notify if this is the active instance
    if (instanceId == m_activeInstanceId) {
        notifyInstanceChanged(it->second);
    }

    return true;
}

bool RobotInstanceManager::createInstance(const std::string& instanceId,
                                           const std::string& modelId) {
    if (m_instances.find(instanceId) != m_instances.end()) {
        LOG_ERROR("Instance already exists: {}", instanceId);
        return false;
    }

    auto model = m_catalog.loadModel(modelId);
    if (!model) {
        LOG_ERROR("Model not found: {}", modelId);
        return false;
    }

    RobotInstance instance;
    instance.instanceId = instanceId;
    instance.baseModelId = modelId;
    instance.created = "now";  // TODO: proper timestamp
    instance.lastModified = instance.created;
    instance.active = m_instances.empty();  // First instance is active
    instance.config = model->toRobotConfig();

    m_instances[instanceId] = instance;

    if (instance.active) {
        m_activeInstanceId = instanceId;
    }

    LOG_INFO("Created new instance: {} (model: {})", instanceId, modelId);
    return true;
}

bool RobotInstanceManager::saveInstance(const std::string& instanceId) {
    auto it = m_instances.find(instanceId);
    if (it == m_instances.end()) {
        LOG_ERROR("Instance not found: {}", instanceId);
        return false;
    }

    const auto& instance = it->second;
    auto instanceDir = m_instancesDir / instanceId;

    try {
        // Create directory if needed
        std::filesystem::create_directories(instanceDir);

        // Save instance.yaml
        YAML::Emitter out;
        out << YAML::BeginMap;
        out << YAML::Key << "instance_id" << YAML::Value << instance.instanceId;
        out << YAML::Key << "base_model" << YAML::Value << instance.baseModelId;
        out << YAML::Key << "created" << YAML::Value << instance.created;
        out << YAML::Key << "last_modified" << YAML::Value << instance.lastModified;
        out << YAML::Key << "active" << YAML::Value << instance.active;

        if (instance.hasTcpOverride) {
            out << YAML::Key << "overrides" << YAML::BeginMap;
            out << YAML::Key << "tcp_offset" << YAML::BeginMap;
            out << YAML::Key << "x" << YAML::Value << instance.tcpOverride.x;
            out << YAML::Key << "y" << YAML::Value << instance.tcpOverride.y;
            out << YAML::Key << "z" << YAML::Value << instance.tcpOverride.z;
            out << YAML::Key << "rx" << YAML::Value << instance.tcpOverride.rx;
            out << YAML::Key << "ry" << YAML::Value << instance.tcpOverride.ry;
            out << YAML::Key << "rz" << YAML::Value << instance.tcpOverride.rz;
            out << YAML::EndMap;
            out << YAML::EndMap;
        }

        out << YAML::EndMap;

        std::ofstream fout(instanceDir / "instance.yaml");
        fout << out.c_str();
        fout.close();

        LOG_INFO("Saved instance: {}", instanceId);
        return true;

    } catch (const std::exception& e) {
        LOG_ERROR("Error saving instance {}: {}", instanceId, e.what());
        return false;
    }
}

bool RobotInstanceManager::saveActiveInstance() {
    if (m_activeInstanceId.empty()) {
        return false;
    }
    return saveInstance(m_activeInstanceId);
}

std::vector<std::string> RobotInstanceManager::getInstanceIds() const {
    std::vector<std::string> ids;
    ids.reserve(m_instances.size());
    for (const auto& [id, instance] : m_instances) {
        ids.push_back(id);
    }
    return ids;
}

bool RobotInstanceManager::hasInstance(const std::string& instanceId) const {
    return m_instances.find(instanceId) != m_instances.end();
}

std::optional<RobotInstance> RobotInstanceManager::getInstance(const std::string& instanceId) const {
    auto it = m_instances.find(instanceId);
    if (it != m_instances.end()) {
        return it->second;
    }
    return std::nullopt;
}

void RobotInstanceManager::notifyInstanceChanged(const RobotInstance& instance) {
    if (m_instanceChangedCallback) {
        m_instanceChangedCallback(instance);
    }
}

} // namespace config
} // namespace robot_controller
