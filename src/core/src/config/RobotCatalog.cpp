/**
 * @file RobotCatalog.cpp
 * @brief Robot Catalog Manager implementation
 */

#include "RobotCatalog.hpp"
#include "../logging/Logger.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>

namespace robot_controller {
namespace config {

bool RobotCatalog::loadCatalog(const std::filesystem::path& catalogDir) {
    m_catalogDir = catalogDir;
    m_models.clear();
    m_loaded = false;

    if (!std::filesystem::exists(catalogDir)) {
        LOG_ERROR("Catalog directory does not exist: {}", catalogDir.string());
        return false;
    }

    auto indexPath = catalogDir / "catalog_index.yaml";
    if (!loadCatalogIndex(indexPath)) {
        LOG_ERROR("Failed to load catalog index: {}", indexPath.string());
        return false;
    }

    m_loaded = true;
    LOG_INFO("Robot catalog loaded: {} models available", m_models.size());
    return true;
}

bool RobotCatalog::loadCatalogIndex(const std::filesystem::path& indexPath) {
    try {
        if (!std::filesystem::exists(indexPath)) {
            LOG_ERROR("Catalog index not found: {}", indexPath.string());
            return false;
        }

        YAML::Node root = YAML::LoadFile(indexPath.string());

        // Get default model
        if (root["default_model"]) {
            m_defaultModelId = root["default_model"].as<std::string>();
        }

        // Load model entries
        if (!root["models"] || !root["models"].IsSequence()) {
            LOG_ERROR("Catalog index missing 'models' array");
            return false;
        }

        for (const auto& entry : root["models"]) {
            std::string id = entry["id"].as<std::string>();
            std::string name = entry["name"].as<std::string>();
            std::string path = entry["path"].as<std::string>();

            // Load model info from model directory
            auto modelDir = m_catalogDir / path;
            RobotModelInfo info;
            info.id = id;
            info.name = name;
            info.catalogPath = modelDir;

            if (loadModelInfo(modelDir, info)) {
                m_models[id] = info;
                LOG_DEBUG("Loaded model info: {} ({})", id, name);
            } else {
                LOG_WARN("Failed to load model info for: {}", id);
            }
        }

        // Set default if not specified
        if (m_defaultModelId.empty() && !m_models.empty()) {
            m_defaultModelId = m_models.begin()->first;
        }

        return !m_models.empty();

    } catch (const YAML::Exception& e) {
        LOG_ERROR("YAML error loading catalog index: {}", e.what());
        return false;
    } catch (const std::exception& e) {
        LOG_ERROR("Error loading catalog index: {}", e.what());
        return false;
    }
}

bool RobotCatalog::loadModelInfo(const std::filesystem::path& modelDir, RobotModelInfo& info) const {
    try {
        auto infoPath = modelDir / "info.yaml";
        if (!std::filesystem::exists(infoPath)) {
            LOG_ERROR("Model info.yaml not found: {}", infoPath.string());
            return false;
        }

        YAML::Node root = YAML::LoadFile(infoPath.string());

        // Load info fields (id and name already set from index)
        if (root["manufacturer"]) info.manufacturer = root["manufacturer"].as<std::string>();
        if (root["model"]) info.model = root["model"].as<std::string>();
        if (root["type"]) info.type = root["type"].as<std::string>();
        if (root["dof"]) info.dof = root["dof"].as<int>();
        if (root["max_payload_kg"]) info.maxPayloadKg = root["max_payload_kg"].as<double>();
        if (root["reach_mm"]) info.reachMm = root["reach_mm"].as<double>();
        if (root["dh_convention"]) info.dhConvention = root["dh_convention"].as<std::string>();

        if (root["mounting"] && root["mounting"].IsSequence()) {
            for (const auto& m : root["mounting"]) {
                info.mounting.push_back(m.as<std::string>());
            }
        }

        return true;

    } catch (const std::exception& e) {
        LOG_ERROR("Error loading model info from {}: {}", modelDir.string(), e.what());
        return false;
    }
}

bool RobotCatalog::loadModelKinematics(const std::filesystem::path& modelDir, RobotModel& model) const {
    try {
        auto kinPath = modelDir / "kinematic.yaml";
        if (!std::filesystem::exists(kinPath)) {
            LOG_ERROR("Model kinematic.yaml not found: {}", kinPath.string());
            return false;
        }

        YAML::Node root = YAML::LoadFile(kinPath.string());

        // Load DH parameters
        if (root["dh_parameters"] && root["dh_parameters"].IsSequence()) {
            model.dhParameters.clear();
            for (const auto& dh : root["dh_parameters"]) {
                DHParameter param;
                param.joint = dh["joint"].as<int>();
                param.a = dh["a"].as<double>();
                param.alpha = dh["alpha"].as<double>();
                param.d = dh["d"].as<double>();
                param.theta_offset = dh["theta_offset"].as<double>();
                model.dhParameters.push_back(param);
            }
        }

        // Load default TCP
        if (root["default_tcp"]) {
            auto tcp = root["default_tcp"];
            model.defaultTcp.x = tcp["x"].as<double>(0.0);
            model.defaultTcp.y = tcp["y"].as<double>(0.0);
            model.defaultTcp.z = tcp["z"].as<double>(0.0);
            model.defaultTcp.rx = tcp["rx"].as<double>(0.0);
            model.defaultTcp.ry = tcp["ry"].as<double>(0.0);
            model.defaultTcp.rz = tcp["rz"].as<double>(0.0);
        }

        // Load home position
        if (root["home_position"] && root["home_position"].IsSequence()) {
            for (size_t i = 0; i < 6 && i < root["home_position"].size(); ++i) {
                model.homePosition[i] = root["home_position"][i].as<double>();
            }
        }

        return model.dhParameters.size() == 6;

    } catch (const std::exception& e) {
        LOG_ERROR("Error loading kinematics from {}: {}", modelDir.string(), e.what());
        return false;
    }
}

bool RobotCatalog::loadModelLimits(const std::filesystem::path& modelDir, RobotModel& model) const {
    try {
        auto limitsPath = modelDir / "limits.yaml";
        if (!std::filesystem::exists(limitsPath)) {
            LOG_ERROR("Model limits.yaml not found: {}", limitsPath.string());
            return false;
        }

        YAML::Node root = YAML::LoadFile(limitsPath.string());

        // Load joint limits
        if (root["joint_limits"] && root["joint_limits"].IsSequence()) {
            model.jointLimits.clear();
            for (const auto& jl : root["joint_limits"]) {
                JointLimit limit;
                limit.joint = jl["joint"].as<int>();
                limit.min = jl["min_deg"].as<double>();
                limit.max = jl["max_deg"].as<double>();
                limit.max_velocity = jl["max_velocity_deg_s"].as<double>();
                limit.max_acceleration = jl["max_acceleration_deg_s2"].as<double>();
                model.jointLimits.push_back(limit);
            }
        }

        return model.jointLimits.size() == 6;

    } catch (const std::exception& e) {
        LOG_ERROR("Error loading limits from {}: {}", modelDir.string(), e.what());
        return false;
    }
}

std::vector<RobotModelInfo> RobotCatalog::getAvailableModels() const {
    std::vector<RobotModelInfo> result;
    result.reserve(m_models.size());
    for (const auto& [id, info] : m_models) {
        result.push_back(info);
    }
    return result;
}

std::vector<std::string> RobotCatalog::getModelIds() const {
    std::vector<std::string> ids;
    ids.reserve(m_models.size());
    for (const auto& [id, info] : m_models) {
        ids.push_back(id);
    }
    return ids;
}

std::optional<RobotModel> RobotCatalog::loadModel(const std::string& modelId) const {
    auto it = m_models.find(modelId);
    if (it == m_models.end()) {
        LOG_ERROR("Model not found in catalog: {}", modelId);
        return std::nullopt;
    }

    RobotModel model;
    model.info = it->second;

    // Load kinematic data
    if (!loadModelKinematics(model.info.catalogPath, model)) {
        LOG_ERROR("Failed to load kinematics for model: {}", modelId);
        return std::nullopt;
    }

    // Load limits
    if (!loadModelLimits(model.info.catalogPath, model)) {
        LOG_ERROR("Failed to load limits for model: {}", modelId);
        return std::nullopt;
    }

    LOG_INFO("Loaded robot model: {} ({})", model.info.name, modelId);
    return model;
}

bool RobotCatalog::hasModel(const std::string& modelId) const {
    return m_models.find(modelId) != m_models.end();
}

std::optional<RobotModelInfo> RobotCatalog::getModelInfo(const std::string& modelId) const {
    auto it = m_models.find(modelId);
    if (it != m_models.end()) {
        return it->second;
    }
    return std::nullopt;
}

} // namespace config
} // namespace robot_controller
