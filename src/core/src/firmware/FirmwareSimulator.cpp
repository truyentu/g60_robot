#include "FirmwareSimulator.hpp"
#include "../logging/Logger.hpp"
#include <sstream>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <fstream>

namespace robot_controller {
namespace firmware {

FirmwareSimulator::FirmwareSimulator() {
    m_positions.fill(0.0);
    m_targets.fill(0.0);

    // Create Ruckig smoother with 1ms cycle time
    m_smoother = std::make_unique<jog::RuckigSmoother>(0.001);

    // Set default velocity limits (will be overridden by robot config)
    std::array<double, 6> maxVel = m_maxVelocities;
    std::array<double, 6> maxAccel, maxJerk;
    for (int i = 0; i < 6; i++) {
        maxAccel[i] = maxVel[i] * 3.0;  // accel = 3x max vel
        maxJerk[i] = maxAccel[i] * 5.0; // jerk = 5x max accel
    }
    m_smoother->setAllLimits(maxVel, maxAccel, maxJerk);

    LOG_INFO("FirmwareSimulator created with Ruckig S-curve");
}

bool FirmwareSimulator::sendCommand(const std::string& gcode) {
    {
        std::ofstream dbg("E:/DEV_CONTEXT_PROJECTs/Robot_controller/jog_debug_core.log", std::ios::app);
        dbg << "[FW] sendCommand: '" << gcode << "' state=" << getStateString() << std::endl;
    }
    if (m_state == State::ALARM) {
        m_responses.push("error:9");  // Alarm lock
        return false;
    }

    if (gcode.empty()) return false;

    // Real-time commands (single char)
    if (gcode.size() == 1) {
        char c = gcode[0];
        if (c == '?') {
            requestStatus();
            return true;
        }
        if (c == '\x85') {  // Jog cancel - stop immediately
            std::lock_guard<std::mutex> lock(m_mutex);
            m_targets = m_positions;  // Stop where we are
            m_currentVelocities.fill(0.0);
            if (m_smoother && m_smoother->isActive()) {
                m_smoother->cancelToStop(m_positions, {0,0,0,0,0,0});
            }
            m_isJogMode = false;
            m_state = State::IDLE;
            m_responses.push("ok");
            return true;
        }
        if (c == '!') {  // Feed hold
            m_state = State::HOLD;
            return true;
        }
        if (c == '~') {  // Cycle start / resume
            if (m_state == State::HOLD) {
                m_state = State::RUN;
            }
            return true;
        }
        if (c == '\x18') {  // Soft reset
            reset();
            return true;
        }
    }

    // Jog command: $J=G91 G21 Xval Fval
    if (gcode.substr(0, 3) == "$J=") {
        return parseJogCommand(gcode.substr(3));
    }

    // G-code command
    return parseGcode(gcode);
}

std::string FirmwareSimulator::getResponse() {
    if (m_statusRequested) {
        m_statusRequested = false;
        return generateStatusReport();
    }

    if (!m_responses.empty()) {
        auto resp = m_responses.front();
        m_responses.pop();
        return resp;
    }

    return "";
}

void FirmwareSimulator::update(double dt_seconds) {
    if (m_state == State::IDLE || m_state == State::HOLD || m_state == State::ALARM) {
        return;
    }

    // Use Ruckig smoother if active, otherwise fallback to linear
    if (m_smoother && m_smoother->isActive()) {
        // Ruckig was created with 1ms cycle time, so we need to call update()
        // multiple times to cover the actual dt_seconds elapsed.
        int steps = std::max(1, static_cast<int>(std::round(dt_seconds / 0.001)));

        jog::RuckigSmoother::TrajectoryPoint point;
        bool completed = false;

        for (int s = 0; s < steps && !completed; s++) {
            point = m_smoother->update();
            if (point.isComplete) {
                completed = true;
            }
        }

        {
            std::lock_guard<std::mutex> lock(m_mutex);
            for (int i = 0; i < SIM_NUM_AXES; i++) {
                m_positions[i] = std::clamp(point.position[i], m_minLimits[i], m_maxLimits[i]);
                m_currentVelocities[i] = point.velocity[i];
            }
        }

        if (completed) {
            m_state = State::IDLE;
            m_isJogMode = false;
            m_responses.push("ok");
        }
    } else {
        // Fallback: linear interpolation
        interpolateMotion(dt_seconds);

        if (isAtTarget()) {
            m_state = State::IDLE;
            m_isJogMode = false;
            m_responses.push("ok");
        }
    }
}

std::array<double, SIM_NUM_AXES> FirmwareSimulator::getJointPositions() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_positions;
}

std::string FirmwareSimulator::getStateString() const {
    switch (m_state.load()) {
        case State::IDLE:  return "Idle";
        case State::RUN:   return "Run";
        case State::JOG:   return "Jog";
        case State::HOLD:  return "Hold";
        case State::ALARM: return "Alarm";
        default:           return "Unknown";
    }
}

std::string FirmwareSimulator::generateStatusReport() const {
    std::lock_guard<std::mutex> lock(m_mutex);

    std::ostringstream ss;
    ss << "<" << getStateString() << "|MPos:";
    ss << std::fixed << std::setprecision(3);
    for (int i = 0; i < SIM_NUM_AXES; i++) {
        if (i > 0) ss << ",";
        ss << m_positions[i];
    }
    ss << "|FS:" << static_cast<int>(m_feedRate) << ",0>";

    return ss.str();
}

void FirmwareSimulator::requestStatus() {
    m_statusRequested = true;
}

void FirmwareSimulator::emergencyStop() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_targets = m_positions;
    m_isJogMode = false;
    m_state = State::ALARM;
    LOG_WARN("FirmwareSimulator: Emergency stop!");
}

void FirmwareSimulator::reset() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_targets = m_positions;
    m_isJogMode = false;
    m_state = State::IDLE;
    while (!m_responses.empty()) m_responses.pop();
    LOG_INFO("FirmwareSimulator: Reset");
}

void FirmwareSimulator::setJointLimits(int joint, double minDeg, double maxDeg) {
    if (joint >= 0 && joint < SIM_NUM_AXES) {
        m_minLimits[joint] = minDeg;
        m_maxLimits[joint] = maxDeg;
    }
}

void FirmwareSimulator::setMaxVelocity(int joint, double degPerSec) {
    if (joint >= 0 && joint < SIM_NUM_AXES) {
        m_maxVelocities[joint] = degPerSec;
        if (m_smoother) {
            double accel = degPerSec * 3.0;
            double jerk = accel * 5.0;
            m_smoother->setJointLimits(joint, degPerSec, accel, jerk);
        }
    }
}

void FirmwareSimulator::setJointPositionsDirect(const std::array<double, DRIVER_NUM_AXES>& positionsDeg) {
    std::lock_guard<std::mutex> lock(m_mutex);
    for (int i = 0; i < SIM_NUM_AXES; i++) {
        m_positions[i] = std::clamp(positionsDeg[i], m_minLimits[i], m_maxLimits[i]);
        m_targets[i] = m_positions[i];
    }
    m_currentVelocities.fill(0.0);
    // Cancel any active Ruckig trajectory
    if (m_smoother && m_smoother->isActive()) {
        m_smoother->cancelToStop(m_positions, {0,0,0,0,0,0});
    }
    m_state = State::JOG;
}

bool FirmwareSimulator::parseGcode(const std::string& gcode) {
    std::lock_guard<std::mutex> lock(m_mutex);

    // Parse: G1 J0:45.5 J1:-30.0 F3600
    // Or: G1 X10 Y20 Z30 A40 B50 C60 F3600
    std::istringstream iss(gcode);
    std::string token;

    bool hasTarget = false;
    auto newTargets = m_positions;  // Start from current

    while (iss >> token) {
        if (token == "G0" || token == "G1") {
            // Motion command
            continue;
        }

        // Joint format: J0:angle
        if (token.size() > 2 && token[0] == 'J' && token[2] == ':') {
            int joint = token[1] - '0';
            if (joint >= 0 && joint < SIM_NUM_AXES) {
                double angle = std::stod(token.substr(3));
                angle = std::clamp(angle, m_minLimits[joint], m_maxLimits[joint]);
                newTargets[joint] = angle;
                hasTarget = true;
            }
        }
        // Axis letter format: X, Y, Z, A, B, C
        else if (token.size() > 1) {
            char axis = token[0];
            int idx = -1;
            if (axis == 'X') idx = 0;
            else if (axis == 'Y') idx = 1;
            else if (axis == 'Z') idx = 2;
            else if (axis == 'A') idx = 3;
            else if (axis == 'B') idx = 4;
            else if (axis == 'C') idx = 5;
            else if (axis == 'F') {
                m_feedRate = std::stod(token.substr(1));
                continue;
            }

            if (idx >= 0 && idx < SIM_NUM_AXES) {
                double value = std::stod(token.substr(1));
                value = std::clamp(value, m_minLimits[idx], m_maxLimits[idx]);
                newTargets[idx] = value;
                hasTarget = true;
            }
        }
    }

    if (hasTarget) {
        m_targets = newTargets;
        m_state = State::RUN;

        // Start Ruckig trajectory
        if (m_smoother) {
            m_smoother->setTarget(m_positions, m_currentVelocities, m_targets);
        }
        return true;
    }

    m_responses.push("ok");
    return true;
}

bool FirmwareSimulator::parseJogCommand(const std::string& gcode) {
    std::lock_guard<std::mutex> lock(m_mutex);

    {
        std::ofstream dbg("E:/DEV_CONTEXT_PROJECTs/Robot_controller/jog_debug_core.log", std::ios::app);
        dbg << "[FW] parseJogCommand: '" << gcode << "'" << std::endl;
        dbg << "[FW] current positions: ";
        for (int i = 0; i < SIM_NUM_AXES; i++) dbg << "J" << i << "=" << m_positions[i] << " ";
        dbg << std::endl;
    }

    // Parse: G91 G21 X10.0 F1000
    // or: G91 J0:10.0 F1000
    std::istringstream iss(gcode);
    std::string token;

    bool isRelative = false;
    auto newTargets = m_positions;

    while (iss >> token) {
        if (token == "G91") {
            isRelative = true;
            continue;
        }
        if (token == "G90" || token == "G21" || token == "G20") {
            continue;
        }

        // Joint format
        if (token.size() > 2 && token[0] == 'J' && token[2] == ':') {
            int joint = token[1] - '0';
            if (joint >= 0 && joint < SIM_NUM_AXES) {
                double value = std::stod(token.substr(3));
                double target = isRelative ? m_positions[joint] + value : value;
                target = std::clamp(target, m_minLimits[joint], m_maxLimits[joint]);
                newTargets[joint] = target;
            }
        }
        // Axis letter format
        else if (token.size() > 1) {
            char axis = token[0];
            int idx = -1;
            if (axis == 'X') idx = 0;
            else if (axis == 'Y') idx = 1;
            else if (axis == 'Z') idx = 2;
            else if (axis == 'A') idx = 3;
            else if (axis == 'B') idx = 4;
            else if (axis == 'C') idx = 5;
            else if (axis == 'F') {
                m_feedRate = std::stod(token.substr(1));
                continue;
            }

            if (idx >= 0 && idx < SIM_NUM_AXES) {
                double value = std::stod(token.substr(1));
                double target = isRelative ? m_positions[idx] + value : value;
                target = std::clamp(target, m_minLimits[idx], m_maxLimits[idx]);
                newTargets[idx] = target;
            }
        }
    }

    // Check if this is the same jog target already in progress (continuous jog resend)
    bool sameTarget = (m_state == State::JOG && m_isJogMode);
    if (sameTarget) {
        bool allSame = true;
        for (int i = 0; i < SIM_NUM_AXES; i++) {
            if (std::abs(newTargets[i] - m_targets[i]) > 0.01) {
                allSame = false;
                break;
            }
        }
        if (allSame && m_smoother && m_smoother->isActive()) {
            // Same continuous jog command - don't reset Ruckig, let it keep running
            return true;
        }
    }

    m_targets = newTargets;
    m_isJogMode = true;
    m_state = State::JOG;

    {
        std::ofstream dbg("E:/DEV_CONTEXT_PROJECTs/Robot_controller/jog_debug_core.log", std::ios::app);
        dbg << "[FW] parseJogCommand result: targets=";
        for (int i = 0; i < SIM_NUM_AXES; i++) dbg << "J" << i << "=" << m_targets[i] << " ";
        dbg << " state=JOG" << std::endl;
    }

    // Start Ruckig trajectory for jog
    if (m_smoother) {
        m_smoother->setTarget(m_positions, m_currentVelocities, m_targets);
    }

    return true;
}

void FirmwareSimulator::interpolateMotion(double dt) {
    std::lock_guard<std::mutex> lock(m_mutex);

    double feedRateDegPerSec = m_feedRate / 60.0;

    for (int i = 0; i < SIM_NUM_AXES; i++) {
        double diff = m_targets[i] - m_positions[i];
        if (std::abs(diff) < POSITION_TOLERANCE) {
            m_positions[i] = m_targets[i];
            continue;
        }

        // Velocity for this joint: limited by feedrate and max velocity
        double maxVel = std::min(feedRateDegPerSec, m_maxVelocities[i]);
        double step = maxVel * dt;

        if (std::abs(diff) <= step) {
            m_positions[i] = m_targets[i];
        } else {
            m_positions[i] += (diff > 0 ? step : -step);
        }

        // Clamp to limits
        m_positions[i] = std::clamp(m_positions[i], m_minLimits[i], m_maxLimits[i]);
    }
}

bool FirmwareSimulator::isAtTarget() const {
    // NOTE: called without lock - caller must hold m_mutex or accept races
    for (int i = 0; i < SIM_NUM_AXES; i++) {
        if (std::abs(m_targets[i] - m_positions[i]) > POSITION_TOLERANCE) {
            return false;
        }
    }
    return true;
}

} // namespace firmware
} // namespace robot_controller
