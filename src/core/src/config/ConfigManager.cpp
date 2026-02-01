/**
 * @file ConfigManager.cpp
 * @brief Configuration manager implementation
 */

#include "ConfigManager.hpp"
#include "../logging/Logger.hpp"
#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <filesystem>

namespace robot_controller {
namespace config {

using json = nlohmann::json;
namespace fs = std::filesystem;

ConfigManager& ConfigManager::instance() {
    static ConfigManager instance;
    return instance;
}

bool ConfigManager::loadRobotConfig(const std::string& filepath) {
    std::lock_guard<std::mutex> lock(m_mutex);

    try {
        if (!fs::exists(filepath)) {
            LOG_ERROR("Robot config file not found: {}", filepath);
            return false;
        }

        LOG_INFO("Loading robot config from: {}", filepath);

        YAML::Node config = YAML::LoadFile(filepath);
        YAML::Node robot = config["robot"];

        if (!robot) {
            LOG_ERROR("Missing 'robot' section in config");
            return false;
        }

        // Load basic info
        m_robot_config.name = robot["name"].as<std::string>("Robot6DOF");
        m_robot_config.type = robot["type"].as<std::string>("6-axis-articulated");
        m_robot_config.manufacturer = robot["manufacturer"].as<std::string>("Custom");
        m_robot_config.model = robot["model"].as<std::string>("RC-6DOF-01");

        // Load DH parameters
        m_robot_config.dh_parameters.clear();
        if (robot["dh_parameters"]) {
            for (const auto& dh : robot["dh_parameters"]) {
                DHParameter param;
                param.joint = dh["joint"].as<int>(0);
                param.a = dh["a"].as<double>(0.0);
                param.alpha = dh["alpha"].as<double>(0.0);
                param.d = dh["d"].as<double>(0.0);
                param.theta_offset = dh["theta_offset"].as<double>(0.0);
                m_robot_config.dh_parameters.push_back(param);
            }
        }

        // Load joint limits
        m_robot_config.joint_limits.clear();
        if (robot["joint_limits"]) {
            for (const auto& jl : robot["joint_limits"]) {
                JointLimit limit;
                limit.joint = jl["joint"].as<int>(0);
                limit.min = jl["min"].as<double>(-180.0);
                limit.max = jl["max"].as<double>(180.0);
                limit.max_velocity = jl["max_velocity"].as<double>(100.0);
                limit.max_acceleration = jl["max_acceleration"].as<double>(500.0);
                m_robot_config.joint_limits.push_back(limit);
            }
        }

        // Load TCP offset
        if (robot["tcp_offset"]) {
            auto tcp = robot["tcp_offset"];
            m_robot_config.tcp_offset.x = tcp["x"].as<double>(0.0);
            m_robot_config.tcp_offset.y = tcp["y"].as<double>(0.0);
            m_robot_config.tcp_offset.z = tcp["z"].as<double>(0.0);
            m_robot_config.tcp_offset.rx = tcp["rx"].as<double>(0.0);
            m_robot_config.tcp_offset.ry = tcp["ry"].as<double>(0.0);
            m_robot_config.tcp_offset.rz = tcp["rz"].as<double>(0.0);
        }

        // Load home position
        if (robot["home_position"]) {
            auto home = robot["home_position"];
            for (size_t i = 0; i < 6 && i < home.size(); ++i) {
                m_robot_config.home_position[i] = home[i].as<double>(0.0);
            }
        }

        // Load specifications
        m_robot_config.max_payload_kg = robot["max_payload_kg"].as<double>(6.0);
        m_robot_config.reach_mm = robot["reach_mm"].as<double>(1200.0);

        // Validate
        if (!m_robot_config.isValid()) {
            LOG_ERROR("Robot config validation failed: expected 6 joints, got {} DH params and {} limits",
                      m_robot_config.dh_parameters.size(), m_robot_config.joint_limits.size());
            return false;
        }

        LOG_INFO("Robot config loaded: {} ({} joints)", m_robot_config.name, m_robot_config.numJoints());
        return true;

    } catch (const YAML::Exception& e) {
        LOG_ERROR("YAML parse error in robot config: {}", e.what());
        return false;
    } catch (const std::exception& e) {
        LOG_ERROR("Error loading robot config: {}", e.what());
        return false;
    }
}

bool ConfigManager::loadSystemConfig(const std::string& filepath) {
    std::lock_guard<std::mutex> lock(m_mutex);

    try {
        if (!fs::exists(filepath)) {
            LOG_ERROR("System config file not found: {}", filepath);
            return false;
        }

        LOG_INFO("Loading system config from: {}", filepath);

        YAML::Node config = YAML::LoadFile(filepath);
        YAML::Node system = config["system"];

        if (!system) {
            LOG_ERROR("Missing 'system' section in config");
            return false;
        }

        // Version
        m_system_config.version = system["version"].as<std::string>("1.0.0");

        // IPC settings
        if (system["ipc"]) {
            auto ipc = system["ipc"];
            m_system_config.ipc.rep_port = ipc["rep_port"].as<int>(5555);
            m_system_config.ipc.pub_port = ipc["pub_port"].as<int>(5556);
            m_system_config.ipc.bind_address = ipc["bind_address"].as<std::string>("*");
            m_system_config.ipc.heartbeat_ms = ipc["heartbeat_ms"].as<int>(1000);
        }

        // Logging settings
        if (system["logging"]) {
            auto logging = system["logging"];
            m_system_config.logging.level = logging["level"].as<std::string>("info");
            m_system_config.logging.file = logging["file"].as<std::string>("logs/core.log");
            m_system_config.logging.max_size_mb = logging["max_size_mb"].as<int>(10);
            m_system_config.logging.max_files = logging["max_files"].as<int>(5);
            m_system_config.logging.console_enabled = logging["console_enabled"].as<bool>(true);
            m_system_config.logging.file_enabled = logging["file_enabled"].as<bool>(true);
        }

        // Control settings
        if (system["control"]) {
            auto control = system["control"];
            m_system_config.control.cycle_time_ms = control["cycle_time_ms"].as<int>(4);
            m_system_config.control.interpolation_hz = control["interpolation_hz"].as<int>(1000);
            m_system_config.control.status_publish_hz = control["status_publish_hz"].as<int>(10);
        }

        // Serial settings
        if (system["serial"]) {
            auto serial = system["serial"];
            m_system_config.serial.port = serial["port"].as<std::string>("AUTO");
            m_system_config.serial.baudrate = serial["baudrate"].as<int>(115200);
            m_system_config.serial.timeout_ms = serial["timeout_ms"].as<int>(100);
        }

        // Safety settings
        if (system["safety"]) {
            auto safety = system["safety"];
            m_system_config.safety.e_stop_enabled = safety["e_stop_enabled"].as<bool>(true);
            m_system_config.safety.soft_limits_enabled = safety["soft_limits_enabled"].as<bool>(true);
            m_system_config.safety.collision_check_enabled = safety["collision_check_enabled"].as<bool>(false);
            m_system_config.safety.max_speed_percent = safety["max_speed_percent"].as<int>(100);
        }

        LOG_INFO("System config loaded: version {}", m_system_config.version);
        return true;

    } catch (const YAML::Exception& e) {
        LOG_ERROR("YAML parse error in system config: {}", e.what());
        return false;
    } catch (const std::exception& e) {
        LOG_ERROR("Error loading system config: {}", e.what());
        return false;
    }
}

bool ConfigManager::loadAll(const std::string& config_dir) {
    std::string robot_path = config_dir + "/robot_config.yaml";
    std::string system_path = config_dir + "/system_config.yaml";

    bool robot_ok = loadRobotConfig(robot_path);
    bool system_ok = loadSystemConfig(system_path);

    m_loaded = robot_ok && system_ok;
    return m_loaded;
}

bool ConfigManager::saveRobotConfig(const std::string& filepath) const {
    std::lock_guard<std::mutex> lock(m_mutex);

    try {
        YAML::Emitter out;
        out << YAML::BeginMap;
        out << YAML::Key << "robot" << YAML::Value << YAML::BeginMap;

        // Basic info
        out << YAML::Key << "name" << YAML::Value << m_robot_config.name;
        out << YAML::Key << "type" << YAML::Value << m_robot_config.type;
        out << YAML::Key << "manufacturer" << YAML::Value << m_robot_config.manufacturer;
        out << YAML::Key << "model" << YAML::Value << m_robot_config.model;

        // DH Parameters
        out << YAML::Key << "dh_parameters" << YAML::Value << YAML::BeginSeq;
        for (const auto& dh : m_robot_config.dh_parameters) {
            out << YAML::BeginMap;
            out << YAML::Key << "joint" << YAML::Value << dh.joint;
            out << YAML::Key << "a" << YAML::Value << dh.a;
            out << YAML::Key << "alpha" << YAML::Value << dh.alpha;
            out << YAML::Key << "d" << YAML::Value << dh.d;
            out << YAML::Key << "theta_offset" << YAML::Value << dh.theta_offset;
            out << YAML::EndMap;
        }
        out << YAML::EndSeq;

        // Joint limits
        out << YAML::Key << "joint_limits" << YAML::Value << YAML::BeginSeq;
        for (const auto& jl : m_robot_config.joint_limits) {
            out << YAML::BeginMap;
            out << YAML::Key << "joint" << YAML::Value << jl.joint;
            out << YAML::Key << "min" << YAML::Value << jl.min;
            out << YAML::Key << "max" << YAML::Value << jl.max;
            out << YAML::Key << "max_velocity" << YAML::Value << jl.max_velocity;
            out << YAML::Key << "max_acceleration" << YAML::Value << jl.max_acceleration;
            out << YAML::EndMap;
        }
        out << YAML::EndSeq;

        // TCP offset
        out << YAML::Key << "tcp_offset" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "x" << YAML::Value << m_robot_config.tcp_offset.x;
        out << YAML::Key << "y" << YAML::Value << m_robot_config.tcp_offset.y;
        out << YAML::Key << "z" << YAML::Value << m_robot_config.tcp_offset.z;
        out << YAML::Key << "rx" << YAML::Value << m_robot_config.tcp_offset.rx;
        out << YAML::Key << "ry" << YAML::Value << m_robot_config.tcp_offset.ry;
        out << YAML::Key << "rz" << YAML::Value << m_robot_config.tcp_offset.rz;
        out << YAML::EndMap;

        // Home position
        out << YAML::Key << "home_position" << YAML::Value << YAML::Flow << YAML::BeginSeq;
        for (double angle : m_robot_config.home_position) {
            out << angle;
        }
        out << YAML::EndSeq;

        // Specifications
        out << YAML::Key << "max_payload_kg" << YAML::Value << m_robot_config.max_payload_kg;
        out << YAML::Key << "reach_mm" << YAML::Value << m_robot_config.reach_mm;

        out << YAML::EndMap;  // robot
        out << YAML::EndMap;  // root

        std::ofstream fout(filepath);
        fout << out.c_str();
        fout.close();

        LOG_INFO("Robot config saved to: {}", filepath);
        return true;

    } catch (const std::exception& e) {
        LOG_ERROR("Error saving robot config: {}", e.what());
        return false;
    }
}

std::string ConfigManager::robotConfigToJson() const {
    std::lock_guard<std::mutex> lock(m_mutex);

    json j;
    j["name"] = m_robot_config.name;
    j["type"] = m_robot_config.type;
    j["manufacturer"] = m_robot_config.manufacturer;
    j["model"] = m_robot_config.model;
    j["num_joints"] = m_robot_config.numJoints();
    j["max_payload_kg"] = m_robot_config.max_payload_kg;
    j["reach_mm"] = m_robot_config.reach_mm;

    // DH parameters
    j["dh_parameters"] = json::array();
    for (const auto& dh : m_robot_config.dh_parameters) {
        j["dh_parameters"].push_back({
            {"joint", dh.joint},
            {"a", dh.a},
            {"alpha", dh.alpha},
            {"d", dh.d},
            {"theta_offset", dh.theta_offset}
        });
    }

    // Joint limits
    j["joint_limits"] = json::array();
    for (const auto& jl : m_robot_config.joint_limits) {
        j["joint_limits"].push_back({
            {"joint", jl.joint},
            {"min", jl.min},
            {"max", jl.max},
            {"max_velocity", jl.max_velocity},
            {"max_acceleration", jl.max_acceleration}
        });
    }

    // TCP offset
    j["tcp_offset"] = {
        {"x", m_robot_config.tcp_offset.x},
        {"y", m_robot_config.tcp_offset.y},
        {"z", m_robot_config.tcp_offset.z},
        {"rx", m_robot_config.tcp_offset.rx},
        {"ry", m_robot_config.tcp_offset.ry},
        {"rz", m_robot_config.tcp_offset.rz}
    };

    // Home position
    j["home_position"] = m_robot_config.home_position;

    return j.dump();
}

std::string ConfigManager::systemConfigToJson() const {
    std::lock_guard<std::mutex> lock(m_mutex);

    json j;
    j["version"] = m_system_config.version;

    j["ipc"] = {
        {"rep_port", m_system_config.ipc.rep_port},
        {"pub_port", m_system_config.ipc.pub_port},
        {"bind_address", m_system_config.ipc.bind_address}
    };

    j["logging"] = {
        {"level", m_system_config.logging.level},
        {"file", m_system_config.logging.file}
    };

    j["control"] = {
        {"cycle_time_ms", m_system_config.control.cycle_time_ms},
        {"status_publish_hz", m_system_config.control.status_publish_hz}
    };

    j["safety"] = {
        {"e_stop_enabled", m_system_config.safety.e_stop_enabled},
        {"soft_limits_enabled", m_system_config.safety.soft_limits_enabled},
        {"max_speed_percent", m_system_config.safety.max_speed_percent}
    };

    return j.dump();
}

} // namespace config
} // namespace robot_controller
