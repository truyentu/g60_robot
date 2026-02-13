/**
 * @file GcodeAdapter.cpp
 * @brief G-code to V2 structured command translator
 */

#include "GcodeAdapter.hpp"
#include "../../logging/Logger.hpp"
#include <sstream>
#include <cmath>
#include <algorithm>

namespace robot_controller {
namespace firmware {
namespace plugins {

GcodeAdapter::GcodeAdapter(std::shared_ptr<IFirmwareDriver> innerDriver)
    : m_inner(std::move(innerDriver)) {
    LOG_INFO("GcodeAdapter created wrapping {}", m_inner->getDriverName());
}

bool GcodeAdapter::sendCommand(const std::string& gcode) {
    if (gcode.empty()) return false;

    // Single-char real-time commands
    if (gcode.size() == 1) {
        return translateRealTimeChar(gcode[0]);
    }

    // Jog command
    if (gcode.substr(0, 3) == "$J=") {
        return translateJogCommand(gcode.substr(3));
    }

    // Homing commands
    if (gcode == "$H") {
        return translateHomingCommand(gcode);
    }

    // G-code motion commands
    return translateMoveCommand(gcode);
}

std::string GcodeAdapter::getResponse() {
    {
        std::lock_guard<std::mutex> lock(m_responseMutex);
        if (!m_responses.empty()) {
            auto resp = m_responses.front();
            m_responses.pop();
            return resp;
        }
    }
    return m_inner->getResponse();
}

bool GcodeAdapter::translateRealTimeChar(char c) {
    switch (c) {
        case '?':
            m_inner->requestStatus();
            return true;
        case '\x85':  // Jog cancel
            m_inner->jogStop();
            {
                std::lock_guard<std::mutex> lock(m_responseMutex);
                m_responses.push("ok");
            }
            return true;
        case '!':  // Feed hold
            m_inner->stopMotion(0);
            return true;
        case '~':  // Cycle start / resume
            return true;
        case '\x18':  // Soft reset
            m_inner->reset();
            return true;
        default:
            return false;
    }
}

bool GcodeAdapter::translateMoveCommand(const std::string& gcode) {
    std::istringstream iss(gcode);
    std::string token;

    bool isRapid = false;
    auto currentPos = m_inner->getJointPositions();
    auto newTargets = currentPos;
    bool hasTarget = false;

    while (iss >> token) {
        if (token == "G0") { isRapid = true; continue; }
        if (token == "G1") { isRapid = false; continue; }
        if (token.empty()) continue;

        char axis = token[0];
        int idx = -1;
        if (axis == 'X') idx = 0;
        else if (axis == 'Y') idx = 1;
        else if (axis == 'Z') idx = 2;
        else if (axis == 'A') idx = 3;
        else if (axis == 'B') idx = 4;
        else if (axis == 'C') idx = 5;
        else if (axis == 'F' && token.size() > 1) {
            m_feedRate = std::stod(token.substr(1));
            continue;
        }
        else if (token.size() > 2 && token[0] == 'J' && token[2] == ':') {
            idx = token[1] - '0';
            if (idx >= 0 && idx < DRIVER_NUM_AXES) {
                newTargets[idx] = std::stod(token.substr(3));
                hasTarget = true;
            }
            continue;
        }

        if (idx >= 0 && idx < DRIVER_NUM_AXES && token.size() > 1) {
            newTargets[idx] = std::stod(token.substr(1));
            hasTarget = true;
        }
    }

    if (!hasTarget) {
        std::lock_guard<std::mutex> lock(m_responseMutex);
        m_responses.push("ok");
        return true;
    }

    // Convert degrees to steps
    std::array<int32_t, DRIVER_NUM_AXES> stepsPos{};
    for (int i = 0; i < DRIVER_NUM_AXES; i++) {
        stepsPos[i] = m_inner->degreesToSteps(i, newTargets[i]);
    }

    // Convert feed rate (deg/min) to speed (steps/ms)
    double feedDegPerSec = m_feedRate / 60.0;
    uint16_t speed = static_cast<uint16_t>(
        std::clamp(feedDegPerSec * m_stepsPerDegree[0] / 1000.0, 1.0, 65535.0));
    uint16_t accel = speed / 2;

    if (isRapid) {
        speed = 65535;  // Max speed for rapid
    }

    bool ok = m_inner->moveAbsolute(stepsPos, speed, accel);
    {
        std::lock_guard<std::mutex> lock(m_responseMutex);
        m_responses.push(ok ? "ok" : "error:15");
    }
    return ok;
}

bool GcodeAdapter::translateJogCommand(const std::string& gcode) {
    std::istringstream iss(gcode);
    std::string token;

    bool isRelative = false;
    int jogAxis = -1;
    double jogDistance = 0;

    while (iss >> token) {
        if (token == "G91") { isRelative = true; continue; }
        if (token == "G90" || token == "G21" || token == "G20") continue;

        char axis = token[0];
        int idx = -1;
        if (axis == 'X') idx = 0;
        else if (axis == 'Y') idx = 1;
        else if (axis == 'Z') idx = 2;
        else if (axis == 'A') idx = 3;
        else if (axis == 'B') idx = 4;
        else if (axis == 'C') idx = 5;
        else if (axis == 'F' && token.size() > 1) {
            m_feedRate = std::stod(token.substr(1));
            continue;
        }
        else if (token.size() > 2 && token[0] == 'J' && token[2] == ':') {
            idx = token[1] - '0';
            if (idx >= 0 && idx < DRIVER_NUM_AXES) {
                jogAxis = idx;
                jogDistance = std::stod(token.substr(3));
            }
            continue;
        }

        if (idx >= 0 && idx < DRIVER_NUM_AXES && token.size() > 1) {
            jogAxis = idx;
            jogDistance = std::stod(token.substr(1));
        }
    }

    if (jogAxis < 0) return false;

    int8_t direction = (jogDistance > 0) ? 1 : -1;
    double feedDegPerSec = m_feedRate / 60.0;
    uint16_t speed = static_cast<uint16_t>(
        std::clamp(feedDegPerSec * m_stepsPerDegree[jogAxis] / 1000.0, 1.0, 65535.0));

    return m_inner->jogStart(static_cast<uint8_t>(jogAxis), direction, speed);
}

bool GcodeAdapter::translateHomingCommand(const std::string& gcode) {
    if (gcode == "$H") {
        return m_inner->homeStart(0x3F, 0, 0);  // Home all axes
    }
    return false;
}

} // namespace plugins
} // namespace firmware
} // namespace robot_controller
