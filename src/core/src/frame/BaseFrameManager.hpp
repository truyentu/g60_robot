#pragma once

#include "FrameTypes.hpp"
#include <map>
#include <vector>
#include <optional>
#include <functional>
#include <mutex>

namespace robot_controller {
namespace frame {

// Forward declaration - will integrate with KinematicsService later
class IKinematicsProvider {
public:
    virtual ~IKinematicsProvider() = default;
    virtual std::array<double, 6> getCurrentJointAngles() const = 0;
    virtual std::array<double, 3> getTcpPosition() const = 0;
};

/**
 * @brief Manages base/workpiece coordinate frames
 *
 * Handles CRUD operations, active frame selection, coordinate transformations,
 * and calibration workflows for base frames.
 */
class BaseFrameManager {
public:
    BaseFrameManager();
    ~BaseFrameManager() = default;

    // Non-copyable
    BaseFrameManager(const BaseFrameManager&) = delete;
    BaseFrameManager& operator=(const BaseFrameManager&) = delete;

    // ========================================================================
    // CRUD Operations
    // ========================================================================

    /**
     * @brief Create a new base frame
     * @param base Base frame data
     * @return true if created successfully
     */
    bool createBase(const BaseFrame& base);

    /**
     * @brief Update an existing base frame
     * @param id Base frame ID
     * @param base Updated data
     * @return true if updated successfully
     */
    bool updateBase(const std::string& id, const BaseFrame& base);

    /**
     * @brief Delete a base frame
     * @param id Base frame ID
     * @return true if deleted successfully
     */
    bool deleteBase(const std::string& id);

    /**
     * @brief Get a base frame by ID
     * @param id Base frame ID
     * @return Optional containing base frame if found
     */
    std::optional<BaseFrame> getBase(const std::string& id) const;

    /**
     * @brief Get all base frames
     * @return Vector of all base frames
     */
    std::vector<BaseFrame> getAllBases() const;

    // ========================================================================
    // Active Base Management
    // ========================================================================

    /**
     * @brief Set the active base frame
     * @param id Base frame ID
     * @return true if set successfully
     */
    bool setActiveBase(const std::string& id);

    /**
     * @brief Get the active base frame
     * @return Optional containing active base if set
     */
    std::optional<BaseFrame> getActiveBase() const;

    /**
     * @brief Get ID of the active base frame
     * @return Active base ID, empty if none
     */
    std::string getActiveBaseId() const;

    // ========================================================================
    // Coordinate Transformation
    // ========================================================================

    /**
     * @brief Transform a point from world coordinates to active base coordinates
     * @param worldFrame Point in world coordinates
     * @return Point in active base coordinates
     */
    Frame transformToBase(const Frame& worldFrame) const;

    /**
     * @brief Transform a point from active base coordinates to world coordinates
     * @param baseFrame Point in base coordinates
     * @return Point in world coordinates
     */
    Frame transformToWorld(const Frame& baseFrame) const;

    /**
     * @brief Get the active base transformation matrix
     * @return 4x4 transformation matrix (world to base)
     */
    Eigen::Matrix4d getActiveBaseMatrix() const;

    /**
     * @brief Get the inverse of active base transformation
     * @return 4x4 transformation matrix (base to world)
     */
    Eigen::Matrix4d getActiveBaseInverseMatrix() const;

    // ========================================================================
    // Calibration
    // ========================================================================

    /**
     * @brief Start base frame calibration
     * @param method Calibration method
     */
    void startCalibration(BaseCalibrationMethod method);

    /**
     * @brief Record a calibration point at current TCP position
     * @param pointIndex Point index (0=origin, 1=x-dir, 2=xy-plane)
     * @param jointAngles Current joint angles
     * @param tcpPosition Current TCP position in world coords
     * @return true if recorded successfully
     */
    bool recordCalibrationPoint(int pointIndex,
                                const std::array<double, 6>& jointAngles,
                                const std::array<double, 3>& tcpPosition);

    /**
     * @brief Calculate base frame from recorded calibration points
     * @return Calculated frame if successful
     */
    std::optional<Frame> calculateBaseFrame();

    /**
     * @brief Cancel ongoing calibration
     */
    void cancelCalibration();

    /**
     * @brief Get current calibration status
     */
    BaseCalibrationStatus getCalibrationStatus() const;

    /**
     * @brief Check if calibration is in progress
     */
    bool isCalibrating() const;

    // ========================================================================
    // Persistence
    // ========================================================================

    /**
     * @brief Save all base frames to a directory
     * @param directory Path to save directory
     * @return true if saved successfully
     */
    bool saveToDirectory(const std::string& directory);

    /**
     * @brief Load base frames from a directory
     * @param directory Path to load from
     * @return true if loaded successfully
     */
    bool loadFromDirectory(const std::string& directory);

    // ========================================================================
    // Callbacks
    // ========================================================================

    using BaseChangedCallback = std::function<void(const std::string& baseId)>;

    /**
     * @brief Set callback for base frame changes
     */
    void setBaseChangedCallback(BaseChangedCallback callback);

private:
    // Base frames storage
    std::map<std::string, BaseFrame> m_bases;
    std::string m_activeBaseId = "world";
    mutable std::mutex m_mutex;

    // Calibration state
    BaseCalibrationStatus m_calibStatus;
    std::vector<CalibrationPointData> m_calibPoints;

    // Callback
    BaseChangedCallback m_baseChangedCallback;

    // Internal helpers
    void initializeDefaultBase();
    int getRequiredPointCount(BaseCalibrationMethod method) const;
};

} // namespace frame
} // namespace robot_controller
