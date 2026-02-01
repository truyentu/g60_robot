#pragma once
// TODO: Implement in IMPL_P1_03
namespace robot_controller {
class ConfigManager {
public:
    static ConfigManager& instance() {
        static ConfigManager instance;
        return instance;
    }
};
}
