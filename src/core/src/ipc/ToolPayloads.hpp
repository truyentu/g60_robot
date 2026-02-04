#pragma once

#include "../tool/ToolTypes.hpp"
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace robot_controller {
namespace ipc {

using json = nlohmann::json;

// ============================================================================
// Tool Data Payloads
// ============================================================================

struct ToolTCPPayload {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double rx = 0.0;
    double ry = 0.0;
    double rz = 0.0;

    static ToolTCPPayload fromToolTCP(const tool::ToolTCP& tcp) {
        return {tcp.x, tcp.y, tcp.z, tcp.rx, tcp.ry, tcp.rz};
    }

    tool::ToolTCP toToolTCP() const {
        return {x, y, z, rx, ry, rz};
    }
};

inline void to_json(json& j, const ToolTCPPayload& p) {
    j = json{{"x", p.x}, {"y", p.y}, {"z", p.z},
             {"rx", p.rx}, {"ry", p.ry}, {"rz", p.rz}};
}

inline void from_json(const json& j, ToolTCPPayload& p) {
    j.at("x").get_to(p.x);
    j.at("y").get_to(p.y);
    j.at("z").get_to(p.z);
    j.at("rx").get_to(p.rx);
    j.at("ry").get_to(p.ry);
    j.at("rz").get_to(p.rz);
}

struct ToolInertiaPayload {
    double mass = 0.0;
    double cogX = 0.0;
    double cogY = 0.0;
    double cogZ = 0.0;

    static ToolInertiaPayload fromToolInertia(const tool::ToolInertia& inertia) {
        return {inertia.mass, inertia.cogX, inertia.cogY, inertia.cogZ};
    }
};

inline void to_json(json& j, const ToolInertiaPayload& p) {
    j = json{{"mass", p.mass}, {"cogX", p.cogX}, {"cogY", p.cogY}, {"cogZ", p.cogZ}};
}

inline void from_json(const json& j, ToolInertiaPayload& p) {
    j.at("mass").get_to(p.mass);
    p.cogX = j.value("cogX", 0.0);
    p.cogY = j.value("cogY", 0.0);
    p.cogZ = j.value("cogZ", 0.0);
}

struct ToolDataPayload {
    std::string id;
    std::string name;
    std::string description;
    ToolTCPPayload tcp;
    ToolInertiaPayload inertia;
    bool isActive = false;

    static ToolDataPayload fromToolData(const tool::ToolData& data) {
        ToolDataPayload p;
        p.id = data.id;
        p.name = data.name;
        p.description = data.description;
        p.tcp = ToolTCPPayload::fromToolTCP(data.tcp);
        p.inertia = ToolInertiaPayload::fromToolInertia(data.inertia);
        p.isActive = data.isActive;
        return p;
    }

    tool::ToolData toToolData() const {
        tool::ToolData data;
        data.id = id;
        data.name = name;
        data.description = description;
        data.tcp = tcp.toToolTCP();
        data.inertia.mass = inertia.mass;
        data.inertia.cogX = inertia.cogX;
        data.inertia.cogY = inertia.cogY;
        data.inertia.cogZ = inertia.cogZ;
        data.isActive = isActive;
        return data;
    }
};

inline void to_json(json& j, const ToolDataPayload& p) {
    j = json{
        {"id", p.id},
        {"name", p.name},
        {"description", p.description},
        {"tcp", p.tcp},
        {"inertia", p.inertia},
        {"isActive", p.isActive}
    };
}

inline void from_json(const json& j, ToolDataPayload& p) {
    j.at("id").get_to(p.id);
    j.at("name").get_to(p.name);
    p.description = j.value("description", "");
    j.at("tcp").get_to(p.tcp);
    if (j.contains("inertia")) {
        j.at("inertia").get_to(p.inertia);
    }
    p.isActive = j.value("isActive", false);
}

// ============================================================================
// Request/Response Payloads
// ============================================================================

struct GetToolListResponse {
    std::vector<ToolDataPayload> tools;
    std::string activeToolId;
};

inline void to_json(json& j, const GetToolListResponse& p) {
    j = json{{"tools", p.tools}, {"activeToolId", p.activeToolId}};
}

struct GetToolRequest {
    std::string toolId;
};

inline void from_json(const json& j, GetToolRequest& p) {
    j.at("toolId").get_to(p.toolId);
}

struct GetToolResponse {
    bool success = false;
    ToolDataPayload tool;
    std::string error;
};

inline void to_json(json& j, const GetToolResponse& p) {
    j = json{{"success", p.success}, {"tool", p.tool}, {"error", p.error}};
}

struct CreateToolRequest {
    ToolDataPayload tool;
};

inline void from_json(const json& j, CreateToolRequest& p) {
    j.at("tool").get_to(p.tool);
}

struct CreateToolResponse {
    bool success = false;
    std::string error;
};

inline void to_json(json& j, const CreateToolResponse& p) {
    j = json{{"success", p.success}, {"error", p.error}};
}

struct UpdateToolRequest {
    std::string toolId;
    ToolDataPayload tool;
};

inline void from_json(const json& j, UpdateToolRequest& p) {
    j.at("toolId").get_to(p.toolId);
    j.at("tool").get_to(p.tool);
}

struct UpdateToolResponse {
    bool success = false;
    std::string error;
};

inline void to_json(json& j, const UpdateToolResponse& p) {
    j = json{{"success", p.success}, {"error", p.error}};
}

struct DeleteToolRequest {
    std::string toolId;
};

inline void from_json(const json& j, DeleteToolRequest& p) {
    j.at("toolId").get_to(p.toolId);
}

struct DeleteToolResponse {
    bool success = false;
    std::string error;
};

inline void to_json(json& j, const DeleteToolResponse& p) {
    j = json{{"success", p.success}, {"error", p.error}};
}

struct SelectToolRequest {
    std::string toolId;
};

inline void from_json(const json& j, SelectToolRequest& p) {
    j.at("toolId").get_to(p.toolId);
}

struct SelectToolResponse {
    bool success = false;
    std::string error;
};

inline void to_json(json& j, const SelectToolResponse& p) {
    j = json{{"success", p.success}, {"error", p.error}};
}

struct GetActiveToolResponse {
    bool success = false;
    ToolDataPayload tool;
    std::string error;
};

inline void to_json(json& j, const GetActiveToolResponse& p) {
    j = json{{"success", p.success}, {"tool", p.tool}, {"error", p.error}};
}

// ============================================================================
// Calibration Payloads
// ============================================================================

struct StartCalibrationRequest {
    std::string method;  // "FOUR_POINT", "SIX_POINT"
};

inline void from_json(const json& j, StartCalibrationRequest& p) {
    j.at("method").get_to(p.method);
}

struct StartCalibrationResponse {
    bool success = false;
    int pointsRequired = 0;
    std::string error;
};

inline void to_json(json& j, const StartCalibrationResponse& p) {
    j = json{{"success", p.success}, {"pointsRequired", p.pointsRequired}, {"error", p.error}};
}

struct RecordPointRequest {
    std::vector<double> jointAngles;  // Current joint positions
};

inline void from_json(const json& j, RecordPointRequest& p) {
    j.at("jointAngles").get_to(p.jointAngles);
}

struct RecordPointResponse {
    bool success = false;
    int pointIndex = 0;
    int totalRequired = 0;
    bool isComplete = false;
    std::string error;
};

inline void to_json(json& j, const RecordPointResponse& p) {
    j = json{
        {"success", p.success},
        {"pointIndex", p.pointIndex},
        {"totalRequired", p.totalRequired},
        {"isComplete", p.isComplete},
        {"error", p.error}
    };
}

struct FinishCalibrationResponse {
    bool success = false;
    ToolTCPPayload calculatedTcp;
    double residualError = 0.0;
    std::string error;
};

inline void to_json(json& j, const FinishCalibrationResponse& p) {
    j = json{
        {"success", p.success},
        {"calculatedTcp", p.calculatedTcp},
        {"residualError", p.residualError},
        {"error", p.error}
    };
}

struct CalibrationStatusResponse {
    std::string state;  // "IDLE", "COLLECTING_POINTS", etc.
    std::string method;
    int pointsRecorded = 0;
    int pointsRequired = 0;
    std::string error;
};

inline void to_json(json& j, const CalibrationStatusResponse& p) {
    j = json{
        {"state", p.state},
        {"method", p.method},
        {"pointsRecorded", p.pointsRecorded},
        {"pointsRequired", p.pointsRequired},
        {"error", p.error}
    };
}

// ============================================================================
// Event Payloads
// ============================================================================

struct ToolChangedEvent {
    std::string toolId;
    std::string toolName;
};

inline void to_json(json& j, const ToolChangedEvent& p) {
    j = json{{"toolId", p.toolId}, {"toolName", p.toolName}};
}

} // namespace ipc
} // namespace robot_controller
