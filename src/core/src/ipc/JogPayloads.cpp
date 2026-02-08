#include "JogPayloads.hpp"

namespace robot_controller {
namespace ipc {

std::string jogModeToString(JogMode mode) {
    switch (mode) {
        case JogMode::JOINT:     return "JOINT";
        case JogMode::CARTESIAN: return "CARTESIAN";
        default:                 return "UNKNOWN";
    }
}

std::string jogTypeToString(JogType type) {
    switch (type) {
        case JogType::CONTINUOUS:   return "CONTINUOUS";
        case JogType::INCREMENTAL:  return "INCREMENTAL";
        default:                    return "UNKNOWN";
    }
}

std::string jogFrameToString(JogFrame frame) {
    switch (frame) {
        case JogFrame::WORLD: return "WORLD";
        case JogFrame::BASE:  return "BASE";
        case JogFrame::TOOL:  return "TOOL";
        case JogFrame::USER1: return "USER1";
        case JogFrame::USER2: return "USER2";
        default:              return "UNKNOWN";
    }
}

} // namespace ipc
} // namespace robot_controller
