/**
 * @file OverrideManager.hpp
 * @brief Triple override control (KUKA-inspired)
 */

#pragma once

#include <atomic>
#include <algorithm>

namespace robot_controller {
namespace override {

/**
 * Override context for velocity calculation
 */
enum class OverrideContext {
    PROGRAM,    // Program execution
    JOG,        // Manual jog
    MANUAL      // T1 mode limit
};

/**
 * Triple Override Manager (KUKA-inspired)
 *
 * Manages three types of speed override:
 * - Program Override: Controls program execution speed (1-100%)
 * - Jog Override: Controls manual jog speed (1-100%)
 * - Manual Override: T1 mode velocity cap (1-100% of 250mm/s)
 *
 * Velocity Calculation:
 * - T1 Mode: min(BaseVel × Override%, 250 × ManualOverride%)
 * - T2/AUTO Mode: BaseVel × Override%
 */
class OverrideManager {
public:
    OverrideManager() = default;
    ~OverrideManager() = default;

    // Non-copyable
    OverrideManager(const OverrideManager&) = delete;
    OverrideManager& operator=(const OverrideManager&) = delete;

    /**
     * Set program override percentage
     * @param percent Value 1-100
     */
    void setProgramOverride(int percent) {
        m_programOverride = std::clamp(percent, 1, 100);
    }

    /**
     * Set jog override percentage
     * @param percent Value 1-100
     */
    void setJogOverride(int percent) {
        m_jogOverride = std::clamp(percent, 1, 100);
    }

    /**
     * Set manual (T1) override percentage
     * @param percent Value 1-100
     */
    void setManualOverride(int percent) {
        m_manualOverride = std::clamp(percent, 1, 100);
    }

    /**
     * Get program override percentage
     */
    int getProgramOverride() const { return m_programOverride.load(); }

    /**
     * Get jog override percentage
     */
    int getJogOverride() const { return m_jogOverride.load(); }

    /**
     * Get manual override percentage
     */
    int getManualOverride() const { return m_manualOverride.load(); }

    /**
     * Apply override to base velocity
     * @param baseVelocity Base velocity in mm/s or deg/s
     * @param ctx Override context
     * @param isT1Mode True if in T1 (manual) mode
     * @return Effective velocity after override
     */
    double applyOverride(double baseVelocity, OverrideContext ctx, bool isT1Mode = false) const {
        double overridePercent;

        switch (ctx) {
            case OverrideContext::PROGRAM:
                overridePercent = m_programOverride.load() / 100.0;
                break;
            case OverrideContext::JOG:
                overridePercent = m_jogOverride.load() / 100.0;
                break;
            case OverrideContext::MANUAL:
                overridePercent = m_manualOverride.load() / 100.0;
                break;
            default:
                overridePercent = 1.0;
        }

        double effective = baseVelocity * overridePercent;

        // In T1 mode, cap at 250mm/s × manual override
        if (isT1Mode) {
            double maxT1Velocity = 250.0 * (m_manualOverride.load() / 100.0);
            effective = std::min(effective, maxT1Velocity);
        }

        return effective;
    }

    /**
     * Get effective velocity for jog operation
     * @param baseVelocity Base jog velocity
     * @param isT1Mode True if in T1 mode
     * @return Effective jog velocity
     */
    double getEffectiveJogVelocity(double baseVelocity, bool isT1Mode) const {
        return applyOverride(baseVelocity, OverrideContext::JOG, isT1Mode);
    }

    /**
     * Get effective velocity for program execution
     * @param baseVelocity Programmed velocity
     * @param isT1Mode True if in T1 mode
     * @return Effective program velocity
     */
    double getEffectiveProgramVelocity(double baseVelocity, bool isT1Mode) const {
        return applyOverride(baseVelocity, OverrideContext::PROGRAM, isT1Mode);
    }

private:
    std::atomic<int> m_programOverride{100};
    std::atomic<int> m_jogOverride{100};
    std::atomic<int> m_manualOverride{100};
};

} // namespace override
} // namespace robot_controller
