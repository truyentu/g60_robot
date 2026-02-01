#pragma once

#include <vector>
#include <array>
#include <cstdint>
#include <string>
#include <chrono>
#include <memory>
#include <functional>
#include <limits>
#include <cmath>

namespace robot_controller::vision {

// ============================================================================
// Basic Types
// ============================================================================

/// 3D point with intensity
struct Point3D {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
    float intensity{0.0f};

    Point3D() = default;
    Point3D(float x_, float y_, float z_, float i_ = 0.0f)
        : x(x_), y(y_), z(z_), intensity(i_) {}
};

/// 2D point (for profile data)
struct Point2D {
    float x{0.0f};
    float z{0.0f};  // Depth
    float intensity{0.0f};
    bool valid{true};

    Point2D() = default;
    Point2D(float x_, float z_, float i_ = 0.0f, bool v = true)
        : x(x_), z(z_), intensity(i_), valid(v) {}
};

/// Robot pose at acquisition time
struct RobotPose {
    std::array<double, 6> joints{};     // Joint angles (rad)
    std::array<double, 3> position{};   // TCP position (mm)
    std::array<double, 3> orientation{}; // TCP orientation RPY (rad)
    std::array<double, 16> flange{};    // 4x4 flange matrix (row-major)
    uint64_t timestamp{0};
    bool valid{false};
};

// ============================================================================
// Sensor Configuration
// ============================================================================

/// Laser profiler configuration
struct LaserProfilerConfig {
    // Connection
    std::string deviceId;           // Device serial or IP
    std::string interfaceName;      // Network interface

    // Acquisition
    uint32_t profileWidth{2048};    // Profile points per line
    float exposureTime{1000.0f};    // Microseconds
    float gain{1.0f};
    bool autoExposure{false};

    // Trigger
    enum class TriggerMode {
        FreeRun,        // Continuous acquisition
        Software,       // Software trigger
        HardwareEncoder, // Encoder pulse trigger (recommended)
        HardwareLine    // External line trigger
    };
    TriggerMode triggerMode{TriggerMode::FreeRun};
    uint32_t encoderDivider{1};     // Trigger every N encoder pulses
    uint32_t triggerLine{0};        // Hardware trigger line number

    // Filtering
    float intensityThresholdLow{10.0f};
    float intensityThresholdHigh{250.0f};
    bool enableIntensityFilter{true};
    bool enableMedianFilter{false};
    uint32_t medianFilterSize{3};

    // Calibration
    bool applyCalibration{true};
    float scaleX{1.0f};
    float scaleZ{1.0f};
    float offsetX{0.0f};
    float offsetZ{0.0f};
};

/// Area camera configuration
struct CameraConfig {
    std::string deviceId;
    uint32_t width{1920};
    uint32_t height{1080};
    float exposureTime{10000.0f};
    float gain{1.0f};
    bool autoExposure{true};

    enum class TriggerMode {
        FreeRun,
        Software,
        Hardware
    };
    TriggerMode triggerMode{TriggerMode::FreeRun};

    // Bandpass filter wavelength (for arc monitoring)
    float filterWavelength{450.0f};  // nm (Blue for arc rejection)
};

/// Depth camera configuration
struct DepthCameraConfig {
    std::string serialNumber;
    uint32_t depthWidth{1280};
    uint32_t depthHeight{720};
    uint32_t colorWidth{1920};
    uint32_t colorHeight{1080};
    uint32_t frameRate{30};
    bool enableColor{true};
    bool enableDepth{true};
    bool enablePointCloud{true};
    float minDepth{0.1f};   // meters
    float maxDepth{3.0f};   // meters
};

// ============================================================================
// Profile Data
// ============================================================================

/// Single laser profile (2D line scan)
struct LaserProfile {
    std::vector<Point2D> points;    // Profile points
    uint64_t timestamp;             // Acquisition timestamp (ns)
    uint64_t frameId;               // Frame sequence number
    uint32_t encoderPosition;       // Encoder count at acquisition
    RobotPose robotPose;            // Robot pose when acquired
    bool valid{false};

    // Computed properties
    float minZ{0.0f};
    float maxZ{0.0f};
    float avgIntensity{0.0f};
    uint32_t validPointCount{0};

    void computeStats() {
        if (points.empty()) return;

        minZ = std::numeric_limits<float>::max();
        maxZ = std::numeric_limits<float>::lowest();
        float sumI = 0.0f;
        validPointCount = 0;

        for (const auto& p : points) {
            if (p.valid) {
                minZ = std::min(minZ, p.z);
                maxZ = std::max(maxZ, p.z);
                sumI += p.intensity;
                validPointCount++;
            }
        }

        avgIntensity = validPointCount > 0 ? sumI / validPointCount : 0.0f;
    }
};

/// Profile batch (multiple profiles for 3D reconstruction)
struct ProfileBatch {
    std::vector<LaserProfile> profiles;
    uint64_t startTimestamp{0};
    uint64_t endTimestamp{0};
    float scanLength{0.0f};         // Total scan distance (mm)

    size_t totalPoints() const {
        size_t count = 0;
        for (const auto& p : profiles) {
            count += p.validPointCount;
        }
        return count;
    }
};

// ============================================================================
// Point Cloud Data
// ============================================================================

/// Point cloud with organized structure
struct PointCloud {
    std::vector<Point3D> points;
    uint32_t width{0};              // 0 = unorganized
    uint32_t height{0};             // 0 = unorganized
    uint64_t timestamp{0};
    std::string frameId;            // Coordinate frame
    bool isDense{false};            // true = no invalid points

    // Bounding box
    Point3D minBound;
    Point3D maxBound;

    void computeBounds() {
        if (points.empty()) return;

        minBound = {std::numeric_limits<float>::max(),
                    std::numeric_limits<float>::max(),
                    std::numeric_limits<float>::max(), 0};
        maxBound = {std::numeric_limits<float>::lowest(),
                    std::numeric_limits<float>::lowest(),
                    std::numeric_limits<float>::lowest(), 0};

        for (const auto& p : points) {
            minBound.x = std::min(minBound.x, p.x);
            minBound.y = std::min(minBound.y, p.y);
            minBound.z = std::min(minBound.z, p.z);
            maxBound.x = std::max(maxBound.x, p.x);
            maxBound.y = std::max(maxBound.y, p.y);
            maxBound.z = std::max(maxBound.z, p.z);
        }
    }

    size_t size() const { return points.size(); }
    bool empty() const { return points.empty(); }
    bool isOrganized() const { return width > 0 && height > 0; }
};

// ============================================================================
// Camera Frame Data
// ============================================================================

/// Image frame from area camera
struct CameraFrame {
    std::vector<uint8_t> data;
    uint32_t width{0};
    uint32_t height{0};
    uint32_t channels{1};           // 1=Mono, 3=RGB, 4=RGBA
    uint32_t bytesPerPixel{1};
    uint64_t timestamp{0};
    uint64_t frameId{0};

    enum class PixelFormat {
        Mono8,
        Mono16,
        RGB8,
        BGR8,
        BayerRG8,
        BayerBG8
    };
    PixelFormat format{PixelFormat::Mono8};

    size_t dataSize() const {
        return width * height * bytesPerPixel;
    }
};

/// Depth frame with optional color
struct DepthFrame {
    std::vector<uint16_t> depthData;    // Depth in mm
    std::vector<uint8_t> colorData;     // RGB color (optional)
    uint32_t width{0};
    uint32_t height{0};
    float depthScale{0.001f};           // Depth unit in meters
    uint64_t timestamp{0};
    uint64_t frameId{0};

    // Intrinsics
    struct Intrinsics {
        float fx{0}, fy{0};     // Focal length
        float cx{0}, cy{0};     // Principal point
        float k1{0}, k2{0}, k3{0}; // Distortion
        float p1{0}, p2{0};
    };
    Intrinsics depthIntrinsics;
    Intrinsics colorIntrinsics;
};

// ============================================================================
// Sensor Status
// ============================================================================

/// Sensor connection status
enum class SensorStatus {
    Disconnected,
    Connecting,
    Connected,
    Streaming,
    Error,
    NotFound
};

/// Sensor information
struct SensorInfo {
    std::string deviceId;
    std::string serialNumber;
    std::string modelName;
    std::string firmwareVersion;
    std::string manufacturer;
    SensorStatus status{SensorStatus::Disconnected};
    std::string lastError;

    // Performance stats
    float frameRate{0.0f};
    uint64_t framesReceived{0};
    uint64_t framesDropped{0};
    float dataRate{0.0f};       // MB/s
};

// ============================================================================
// Hand-Eye Calibration
// ============================================================================

/// Hand-Eye calibration data (sensor to flange transform)
struct HandEyeCalibration {
    std::array<double, 16> sensorToFlange{};  // 4x4 matrix (row-major)
    std::array<double, 16> flangeToSensor{};  // Inverse transform
    double reprojectionError{0.0};
    bool valid{false};
    std::string calibrationDate;

    // Individual components
    std::array<double, 3> translation{};      // mm
    std::array<double, 4> rotationQuat{};     // Quaternion (w,x,y,z)
    std::array<double, 3> rotationRPY{};      // Roll, Pitch, Yaw (rad)
};

// ============================================================================
// Callbacks
// ============================================================================

using ProfileCallback = std::function<void(const LaserProfile&)>;
using FrameCallback = std::function<void(const CameraFrame&)>;
using DepthCallback = std::function<void(const DepthFrame&)>;
using PointCloudCallback = std::function<void(const PointCloud&)>;

} // namespace robot_controller::vision
