#include "HikrobotLaserProfiler.hpp"
#include <spdlog/spdlog.h>
#include <cstring>
#include <cmath>

// Hikrobot MVS SDK headers
// Note: In actual implementation, include proper SDK headers
// #include "MvCameraControl.h"
// For this plan, we define minimal stubs

// Stub definitions for MVS SDK (replace with actual SDK includes)
#define MV_OK 0
#define MV_E_HANDLE -1

namespace robot_controller::vision {

HikrobotLaserProfiler::HikrobotLaserProfiler() {
    spdlog::info("HikrobotLaserProfiler: Initializing");

    // Initialize SDK (one-time)
    // MV_CC_Initialize();
}

HikrobotLaserProfiler::~HikrobotLaserProfiler() {
    if (acquiring_.load()) {
        stopAcquisition();
    }
    if (connected_.load()) {
        disconnect();
    }

    spdlog::info("HikrobotLaserProfiler: Destroyed");
}

std::vector<SensorInfo> HikrobotLaserProfiler::enumerate() {
    std::vector<SensorInfo> devices;

    spdlog::info("Enumerating Hikrobot laser profilers...");

    // MVS SDK enumeration
    // MV_CC_DEVICE_INFO_LIST deviceList;
    // int ret = MV_CC_EnumDevices(MV_GIGE_DEVICE, &deviceList);

    // For each device in list, create SensorInfo
    // This is placeholder implementation

    // Example: Add dummy device for testing
    SensorInfo info;
    info.deviceId = "192.168.1.100";
    info.serialNumber = "DP2025001";
    info.modelName = "MV-DP2025-04H";
    info.manufacturer = "Hikrobot";
    info.status = SensorStatus::Disconnected;
    devices.push_back(info);

    spdlog::info("Found {} laser profiler(s)", devices.size());
    return devices;
}

bool HikrobotLaserProfiler::connect(const std::string& deviceId) {
    if (connected_.load()) {
        spdlog::warn("Already connected, disconnecting first");
        disconnect();
    }

    spdlog::info("Connecting to laser profiler: {}", deviceId);

    // MVS SDK connection
    // int ret = MV_CC_CreateHandle(&handle_, deviceInfo);
    // if (ret != MV_OK) return false;
    // ret = MV_CC_OpenDevice(handle_);
    // if (ret != MV_OK) return false;

    // Placeholder: simulate connection
    handle_ = reinterpret_cast<void*>(0x12345678);  // Dummy handle

    // Store device info
    info_.deviceId = deviceId;
    info_.status = SensorStatus::Connected;

    connected_.store(true);
    spdlog::info("Connected to laser profiler: {}", deviceId);

    // Apply default configuration
    configure(config_);

    return true;
}

void HikrobotLaserProfiler::disconnect() {
    if (!connected_.load()) return;

    if (acquiring_.load()) {
        stopAcquisition();
    }

    spdlog::info("Disconnecting from laser profiler");

    // MVS SDK disconnect
    // MV_CC_CloseDevice(handle_);
    // MV_CC_DestroyHandle(handle_);

    handle_ = nullptr;
    connected_.store(false);
    info_.status = SensorStatus::Disconnected;

    spdlog::info("Disconnected");
}

bool HikrobotLaserProfiler::isConnected() const {
    return connected_.load();
}

SensorInfo HikrobotLaserProfiler::getInfo() const {
    SensorInfo info = info_;
    info.frameRate = currentFrameRate_.load();
    info.framesReceived = framesReceived_.load();
    info.framesDropped = framesDropped_.load();
    return info;
}

bool HikrobotLaserProfiler::configure(const LaserProfilerConfig& config) {
    std::lock_guard<std::mutex> lock(configMutex_);

    if (!connected_.load()) {
        spdlog::error("Cannot configure: not connected");
        return false;
    }

    spdlog::info("Configuring laser profiler");

    // Set exposure
    if (!setExposure(config.exposureTime)) {
        spdlog::warn("Failed to set exposure");
    }

    // Set gain
    if (!setGain(config.gain)) {
        spdlog::warn("Failed to set gain");
    }

    // Set trigger mode
    if (!setTriggerMode(config.triggerMode)) {
        spdlog::warn("Failed to set trigger mode");
    }

    // Set encoder divider
    if (config.triggerMode == LaserProfilerConfig::TriggerMode::HardwareEncoder) {
        if (!setEncoderDivider(config.encoderDivider)) {
            spdlog::warn("Failed to set encoder divider");
        }
    }

    config_ = config;
    spdlog::info("Configuration applied");
    return true;
}

LaserProfilerConfig HikrobotLaserProfiler::getConfig() const {
    std::lock_guard<std::mutex> lock(configMutex_);
    return config_;
}

bool HikrobotLaserProfiler::setExposure(float exposureUs) {
    if (!connected_.load()) return false;

    // MVS SDK: MV_CC_SetFloatValue(handle_, "ExposureTime", exposureUs);
    spdlog::debug("Set exposure: {} us", exposureUs);
    return true;
}

bool HikrobotLaserProfiler::setGain(float gain) {
    if (!connected_.load()) return false;

    // MVS SDK: MV_CC_SetFloatValue(handle_, "Gain", gain);
    spdlog::debug("Set gain: {}", gain);
    return true;
}

bool HikrobotLaserProfiler::setTriggerMode(LaserProfilerConfig::TriggerMode mode) {
    if (!connected_.load()) return false;

    switch (mode) {
        case LaserProfilerConfig::TriggerMode::FreeRun:
            // MV_CC_SetEnumValue(handle_, "TriggerMode", 0);
            spdlog::info("Trigger mode: FreeRun");
            break;

        case LaserProfilerConfig::TriggerMode::Software:
            // MV_CC_SetEnumValue(handle_, "TriggerMode", 1);
            // MV_CC_SetEnumValue(handle_, "TriggerSource", 7);  // Software
            spdlog::info("Trigger mode: Software");
            break;

        case LaserProfilerConfig::TriggerMode::HardwareEncoder:
            // MV_CC_SetEnumValue(handle_, "TriggerMode", 1);
            // MV_CC_SetEnumValue(handle_, "TriggerSource", 8);  // ShaftEncoderModuleOut
            // MV_CC_SetEnumValue(handle_, "EncoderSourceA", 0); // Line0
            spdlog::info("Trigger mode: HardwareEncoder");
            break;

        case LaserProfilerConfig::TriggerMode::HardwareLine:
            // MV_CC_SetEnumValue(handle_, "TriggerMode", 1);
            // MV_CC_SetEnumValue(handle_, "TriggerSource", 0);  // Line0
            spdlog::info("Trigger mode: HardwareLine");
            break;
    }

    return true;
}

bool HikrobotLaserProfiler::setEncoderDivider(uint32_t divider) {
    if (!connected_.load()) return false;

    // MVS SDK: MV_CC_SetIntValue(handle_, "EncoderDivider", divider);
    spdlog::info("Encoder divider: {}", divider);
    return true;
}

bool HikrobotLaserProfiler::startAcquisition() {
    if (!connected_.load()) {
        spdlog::error("Cannot start acquisition: not connected");
        return false;
    }

    if (acquiring_.load()) {
        spdlog::warn("Already acquiring");
        return true;
    }

    spdlog::info("Starting acquisition");

    // Clear queue
    {
        std::lock_guard<std::mutex> lock(queueMutex_);
        while (!profileQueue_.empty()) {
            profileQueue_.pop();
        }
    }

    // MVS SDK: MV_CC_StartGrabbing(handle_);

    acquiring_.store(true);
    stopFlag_.store(false);
    framesReceived_.store(0);
    framesDropped_.store(0);
    lastFrameTime_ = std::chrono::steady_clock::now();

    // Start acquisition thread
    acquisitionThread_ = std::thread(&HikrobotLaserProfiler::acquisitionLoop, this);

    info_.status = SensorStatus::Streaming;
    spdlog::info("Acquisition started");
    return true;
}

bool HikrobotLaserProfiler::stopAcquisition() {
    if (!acquiring_.load()) return true;

    spdlog::info("Stopping acquisition");

    stopFlag_.store(true);
    queueCondition_.notify_all();

    if (acquisitionThread_.joinable()) {
        acquisitionThread_.join();
    }

    // MVS SDK: MV_CC_StopGrabbing(handle_);

    acquiring_.store(false);
    info_.status = SensorStatus::Connected;

    spdlog::info("Acquisition stopped");
    return true;
}

bool HikrobotLaserProfiler::isAcquiring() const {
    return acquiring_.load();
}

bool HikrobotLaserProfiler::softwareTrigger() {
    if (!connected_.load() || !acquiring_.load()) return false;

    // MVS SDK: MV_CC_SetCommandValue(handle_, "TriggerSoftware");
    return true;
}

void HikrobotLaserProfiler::acquisitionLoop() {
    spdlog::debug("Acquisition thread started");

    while (!stopFlag_.load()) {
        // MVS SDK: Get frame from camera
        // MV_FRAME_OUT frameInfo = {0};
        // int ret = MV_CC_GetImageBuffer(handle_, &frameInfo, 1000);

        // Placeholder: simulate profile acquisition
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        if (stopFlag_.load()) break;

        // Parse raw data into profile
        LaserProfile profile;
        profile.timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        profile.frameId = framesReceived_.load();

        // Simulate profile data (in real implementation, parse from SDK)
        profile.points.resize(config_.profileWidth);
        for (uint32_t i = 0; i < config_.profileWidth; ++i) {
            profile.points[i].x = static_cast<float>(i) * 0.1f - 100.0f;  // -100 to +100 mm
            profile.points[i].z = 50.0f + 10.0f * std::sin(i * 0.1f);  // Simulated depth
            profile.points[i].intensity = 128.0f;
            profile.points[i].valid = true;
        }

        // Apply filters
        if (config_.enableIntensityFilter) {
            applyIntensityFilter(profile);
        }

        // Apply calibration
        if (config_.applyCalibration) {
            applyCalibration(profile);
        }

        profile.computeStats();
        profile.valid = profile.validPointCount > 0;

        // Update stats
        framesReceived_++;
        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - lastFrameTime_).count();
        if (dt > 0) {
            currentFrameRate_.store(1.0f / dt);
        }
        lastFrameTime_ = now;

        // Invoke callback
        {
            std::lock_guard<std::mutex> lock(callbackMutex_);
            if (callback_) {
                callback_(profile);
            }
        }

        // Add to queue
        {
            std::lock_guard<std::mutex> lock(queueMutex_);
            if (profileQueue_.size() >= MAX_QUEUE_SIZE) {
                profileQueue_.pop();  // Drop oldest
                framesDropped_++;
            }
            profileQueue_.push(std::move(profile));
        }
        queueCondition_.notify_one();
    }

    spdlog::debug("Acquisition thread stopped");
}

bool HikrobotLaserProfiler::getProfile(LaserProfile& profile, uint32_t timeoutMs) {
    std::unique_lock<std::mutex> lock(queueMutex_);

    if (queueCondition_.wait_for(lock, std::chrono::milliseconds(timeoutMs),
            [this]{ return !profileQueue_.empty() || stopFlag_.load(); })) {

        if (!profileQueue_.empty()) {
            profile = std::move(profileQueue_.front());
            profileQueue_.pop();
            return true;
        }
    }

    return false;
}

bool HikrobotLaserProfiler::getProfileBatch(ProfileBatch& batch,
                                             uint32_t count,
                                             uint32_t timeoutMs) {
    batch.profiles.clear();
    batch.profiles.reserve(count);

    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(timeoutMs);

    while (batch.profiles.size() < count) {
        auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now()).count();

        if (remaining <= 0) break;

        LaserProfile profile;
        if (getProfile(profile, static_cast<uint32_t>(remaining))) {
            if (batch.profiles.empty()) {
                batch.startTimestamp = profile.timestamp;
            }
            batch.endTimestamp = profile.timestamp;
            batch.profiles.push_back(std::move(profile));
        }
    }

    return !batch.profiles.empty();
}

void HikrobotLaserProfiler::setProfileCallback(ProfileCallback callback) {
    std::lock_guard<std::mutex> lock(callbackMutex_);
    callback_ = std::move(callback);
}

void HikrobotLaserProfiler::clearProfileCallback() {
    std::lock_guard<std::mutex> lock(callbackMutex_);
    callback_ = nullptr;
}

void HikrobotLaserProfiler::applyIntensityFilter(LaserProfile& profile) {
    for (auto& point : profile.points) {
        if (point.intensity < config_.intensityThresholdLow ||
            point.intensity > config_.intensityThresholdHigh) {
            point.valid = false;
        }
    }
}

void HikrobotLaserProfiler::applyCalibration(LaserProfile& profile) {
    for (auto& point : profile.points) {
        if (point.valid) {
            point.x = point.x * config_.scaleX + config_.offsetX;
            point.z = point.z * config_.scaleZ + config_.offsetZ;
        }
    }
}

bool HikrobotLaserProfiler::loadCalibration(const std::string& filepath) {
    spdlog::info("Loading calibration from: {}", filepath);
    // Load calibration file (JSON or XML format)
    // Parse and apply to config_.scaleX/Z, offsetX/Z
    return true;
}

void HikrobotLaserProfiler::setHandEyeCalibration(const HandEyeCalibration& cal) {
    std::lock_guard<std::mutex> lock(configMutex_);
    handEyeCal_ = cal;
    spdlog::info("Hand-eye calibration set, reprojection error: {:.4f}",
                 cal.reprojectionError);
}

HandEyeCalibration HikrobotLaserProfiler::getHandEyeCalibration() const {
    std::lock_guard<std::mutex> lock(configMutex_);
    return handEyeCal_;
}

std::vector<Point3D> HikrobotLaserProfiler::profileToWorldPoints(
    const LaserProfile& profile,
    const RobotPose& robotPose) const {

    std::vector<Point3D> worldPoints;
    worldPoints.reserve(profile.validPointCount);

    if (!handEyeCal_.valid || !robotPose.valid) {
        // No calibration, return points in sensor frame
        for (const auto& p : profile.points) {
            if (p.valid) {
                worldPoints.emplace_back(p.x, 0.0f, p.z, p.intensity);
            }
        }
        return worldPoints;
    }

    // Transform: P_world = T_base_flange * T_flange_sensor * P_sensor
    // Where T_flange_sensor is the hand-eye calibration

    for (const auto& p : profile.points) {
        if (!p.valid) continue;

        Point3D worldPoint;
        worldPoint.x = p.x;
        worldPoint.y = 0.0f;  // Profile is 2D (X-Z plane in sensor frame)
        worldPoint.z = p.z;
        worldPoint.intensity = p.intensity;

        // Apply sensor to flange transform
        transformToWorld(worldPoint, handEyeCal_.sensorToFlange);

        // Apply flange to world transform
        transformToWorld(worldPoint, robotPose.flange);

        worldPoints.push_back(worldPoint);
    }

    return worldPoints;
}

void HikrobotLaserProfiler::transformToWorld(
    Point3D& point,
    const std::array<double, 16>& matrix) const {

    // 4x4 matrix multiplication (row-major)
    // | m0  m1  m2  m3  |   | x |
    // | m4  m5  m6  m7  | * | y |
    // | m8  m9  m10 m11 |   | z |
    // | m12 m13 m14 m15 |   | 1 |

    double x = point.x;
    double y = point.y;
    double z = point.z;

    point.x = static_cast<float>(matrix[0]*x + matrix[1]*y + matrix[2]*z + matrix[3]);
    point.y = static_cast<float>(matrix[4]*x + matrix[5]*y + matrix[6]*z + matrix[7]);
    point.z = static_cast<float>(matrix[8]*x + matrix[9]*y + matrix[10]*z + matrix[11]);
}

void HikrobotLaserProfiler::resetStats() {
    framesReceived_.store(0);
    framesDropped_.store(0);
    currentFrameRate_.store(0.0f);
}

// Factory function
std::unique_ptr<ILaserProfiler> createLaserProfiler(const std::string& type) {
    if (type == "hikrobot" || type == "default") {
        return std::make_unique<HikrobotLaserProfiler>();
    }
    spdlog::error("Unknown laser profiler type: {}", type);
    return nullptr;
}

} // namespace robot_controller::vision
