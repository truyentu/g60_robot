/**
 * @file RobotCatalog.hpp
 * @brief Robot Catalog Manager - loads and manages robot models from catalog
 */

#pragma once

#include "RobotConfig.hpp"
#include <map>
#include <vector>
#include <optional>
#include <string>
#include <filesystem>

namespace robot_controller {
namespace config {

/**
 * Robot Model metadata (from info.yaml)
 */
struct RobotModelInfo {
    std::string id;                     // e.g., "kuka_kr6_r900"
    std::string name;                   // e.g., "KUKA KR 6 R900"
    std::string manufacturer;           // e.g., "KUKA"
    std::string model;                  // e.g., "KR 6 R900"
    std::string type;                   // e.g., "6-axis-articulated"
    int dof = 6;
    double maxPayloadKg = 0.0;
    double reachMm = 0.0;
    std::vector<std::string> mounting;  // e.g., ["floor", "ceiling", "wall"]
    std::string dhConvention = "modified";
    std::filesystem::path catalogPath;  // Path to model directory
};

/**
 * Full robot model (loaded from catalog)
 */
struct RobotModel {
    RobotModelInfo info;
    std::vector<DHParameter> dhParameters;
    std::vector<JointLimit> jointLimits;
    TcpOffset defaultTcp;
    std::array<double, 6> homePosition = {0, 0, 0, 0, 0, 0};

    /**
     * Convert to RobotConfig (for compatibility with existing code)
     */
    RobotConfig toRobotConfig() const {
        RobotConfig config;
        config.name = info.name;
        config.type = info.type;
        config.manufacturer = info.manufacturer;
        config.model = info.model;
        config.dh_parameters = dhParameters;
        config.joint_limits = jointLimits;
        config.tcp_offset = defaultTcp;
        config.home_position = homePosition;
        config.max_payload_kg = info.maxPayloadKg;
        config.reach_mm = info.reachMm;
        return config;
    }
};

/**
 * Catalog index entry
 */
struct CatalogEntry {
    std::string id;
    std::string name;
    std::string path;
};

/**
 * Robot Catalog Manager
 *
 * Loads and manages robot models from the catalog directory.
 * Each robot model consists of:
 *   - info.yaml: Model metadata
 *   - kinematic.yaml: DH parameters
 *   - limits.yaml: Joint limits
 */
class RobotCatalog {
public:
    RobotCatalog() = default;
    ~RobotCatalog() = default;

    /**
     * Load catalog from directory
     * @param catalogDir Path to catalog directory (containing catalog_index.yaml)
     * @return true if loaded successfully
     */
    bool loadCatalog(const std::filesystem::path& catalogDir);

    /**
     * Get list of available robot models
     * @return Vector of model info (lightweight, doesn't load full model)
     */
    std::vector<RobotModelInfo> getAvailableModels() const;

    /**
     * Get list of model IDs
     */
    std::vector<std::string> getModelIds() const;

    /**
     * Load full robot model by ID
     * @param modelId Model identifier (e.g., "kuka_kr6_r900")
     * @return Optional containing model if found, nullopt otherwise
     */
    std::optional<RobotModel> loadModel(const std::string& modelId) const;

    /**
     * Check if model exists in catalog
     */
    bool hasModel(const std::string& modelId) const;

    /**
     * Get model info by ID (without loading full model)
     */
    std::optional<RobotModelInfo> getModelInfo(const std::string& modelId) const;

    /**
     * Get default model ID
     */
    std::string getDefaultModelId() const { return m_defaultModelId; }

    /**
     * Check if catalog is loaded
     */
    bool isLoaded() const { return m_loaded; }

    /**
     * Get number of models in catalog
     */
    size_t modelCount() const { return m_models.size(); }

private:
    bool loadCatalogIndex(const std::filesystem::path& indexPath);
    bool loadModelInfo(const std::filesystem::path& modelDir, RobotModelInfo& info) const;
    bool loadModelKinematics(const std::filesystem::path& modelDir, RobotModel& model) const;
    bool loadModelLimits(const std::filesystem::path& modelDir, RobotModel& model) const;

    std::filesystem::path m_catalogDir;
    std::map<std::string, RobotModelInfo> m_models;
    std::string m_defaultModelId;
    bool m_loaded = false;
};

} // namespace config
} // namespace robot_controller
