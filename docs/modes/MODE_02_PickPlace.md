# MODE_02: Pick & Place Mode

| Metadata      | Value                           |
|---------------|---------------------------------|
| Module ID     | MODE_02                         |
| Priority      | P1 (Secondary Mode)             |
| Status        | DRAFT                           |
| Dependencies  | CORE_02, CORE_03, CORE_05       |

---

## 1. Tổng quan

### 1.1 Mục đích

Pick & Place mode cho phép robot thực hiện các tác vụ gắp-đặt vật với hỗ trợ vision guidance. Mode này mở rộng khả năng của robot từ ứng dụng hàn sang các ứng dụng material handling.

### 1.2 Use Cases

| Use Case | Mô tả |
|----------|-------|
| **Bin Picking** | Gắp chi tiết từ thùng với vision guidance |
| **Palletizing** | Xếp hàng lên pallet theo pattern |
| **Machine Tending** | Nạp/lấy phôi cho máy CNC |
| **Assembly** | Lắp ráp các component |
| **Sorting** | Phân loại chi tiết theo loại/màu |

### 1.3 Kiến trúc tích hợp Vision

```
┌─────────────────────────────────────────────────────────────────┐
│                         HMI Layer (C#)                          │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐          │
│  │ PickPlaceView│  │ VisionView   │  │ CalibView    │          │
│  └──────────────┘  └──────────────┘  └──────────────┘          │
└───────────────────────────┬─────────────────────────────────────┘
                            │ ZeroMQ (tcp://127.0.0.1:5560)
┌───────────────────────────┴─────────────────────────────────────┐
│                      Core Logic (C++)                           │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐          │
│  │PickPlaceMgr  │  │ GripperCtrl  │  │ VisionBridge │          │
│  └──────────────┘  └──────────────┘  └──────────────┘          │
└───────────────────────────┬─────────────────────────────────────┘
                            │ ZeroMQ (tcp://127.0.0.1:5570)
┌───────────────────────────┴─────────────────────────────────────┐
│                    Vision Process (Python)                      │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐          │
│  │ CameraDriver │  │ ObjectDetect │  │ PoseEstimate │          │
│  └──────────────┘  └──────────────┘  └──────────────┘          │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. Gripper Control

### 2.1 Gripper Types

| Type | Điều khiển | I/O | Use Case |
|------|-----------|-----|----------|
| **Pneumatic 2-Finger** | Van 5/2 | 2 DO + 2 DI | Chi tiết đơn giản |
| **Pneumatic Vacuum** | Ejector + sensor | 1 DO + 1 DI | Tấm phẳng |
| **Electric Servo** | RS485/Modbus | Serial | Chi tiết đa dạng |

### 2.2 I/O Map

```
┌─────────────────────────────────────────────────────┐
│                 GRIPPER I/O MAP                     │
├─────────────────────────────────────────────────────┤
│ Digital Outputs (DO)                                │
│   DO8: GRIPPER_CLOSE       (Lệnh đóng gripper)      │
│   DO9: GRIPPER_OPEN        (Lệnh mở gripper)        │
│   DO10: VACUUM_ON          (Bật vacuum)             │
│   DO11: BLOW_OFF           (Thổi nhả chi tiết)      │
│                                                     │
│ Digital Inputs (DI)                                 │
│   DI8: GRIPPER_CLOSED      (Sensor đóng)            │
│   DI9: GRIPPER_OPENED      (Sensor mở)              │
│   DI10: PART_DETECTED      (Sensor có chi tiết)     │
│   DI11: VACUUM_OK          (Áp suất đủ)             │
├─────────────────────────────────────────────────────┤
│ Analog Inputs (AI)                                  │
│   AI4: GRIPPER_FORCE       (Lực kẹp - 0-10V)        │
│   AI5: VACUUM_PRESSURE     (Áp suất - 0-10V)        │
└─────────────────────────────────────────────────────┘
```

### 2.3 Gripper Controller

```cpp
// GripperController.h
#pragma once
#include <atomic>
#include <chrono>

namespace Motion {

enum class GripperType {
    PNEUMATIC_2FINGER,
    PNEUMATIC_VACUUM,
    ELECTRIC_SERVO
};

enum class GripperState {
    UNKNOWN,
    OPENED,
    CLOSED,
    OPENING,
    CLOSING,
    GRIPPING,  // Có chi tiết
    ERROR
};

struct GripperConfig {
    GripperType type = GripperType::PNEUMATIC_2FINGER;

    // Timing
    uint32_t close_timeout_ms = 500;
    uint32_t open_timeout_ms = 500;
    uint32_t settle_time_ms = 50;

    // Vacuum settings
    double vacuum_threshold_kpa = -60.0;  // Ngưỡng vacuum OK
    uint32_t vacuum_timeout_ms = 1000;
    uint32_t blowoff_time_ms = 100;

    // Electric gripper
    double grip_force_n = 50.0;
    double grip_speed_mm_s = 100.0;
};

struct GripperStatus {
    GripperState state = GripperState::UNKNOWN;
    bool part_detected = false;
    double force_n = 0.0;
    double vacuum_kpa = 0.0;
    bool sensor_closed = false;
    bool sensor_opened = false;
};

class GripperController {
public:
    explicit GripperController(IOInterface& io);

    // Configuration
    void configure(const GripperConfig& config);

    // Commands (blocking)
    bool grip(uint32_t timeout_ms = 0);    // Đóng và xác nhận có chi tiết
    bool release(uint32_t timeout_ms = 0); // Mở và xác nhận đã nhả

    // Commands (non-blocking)
    void startGrip();
    void startRelease();
    bool isActionComplete() const;

    // Status
    GripperStatus getStatus() const;
    bool haspart() const { return status_.part_detected; }
    bool isReady() const { return status_.state == GripperState::OPENED; }

    // Vacuum specific
    void enableVacuum();
    void disableVacuum();
    void blowOff();

private:
    void updateStatus();
    bool waitForState(GripperState target, uint32_t timeout_ms);

    IOInterface& io_;
    GripperConfig config_;
    GripperStatus status_;
    std::atomic<bool> action_complete_{true};
};

} // namespace Motion
```

### 2.4 Gripper Implementation

```cpp
// GripperController.cpp
#include "GripperController.h"
#include <thread>

namespace Motion {

bool GripperController::grip(uint32_t timeout_ms) {
    if (timeout_ms == 0) {
        timeout_ms = config_.close_timeout_ms;
    }

    switch (config_.type) {
        case GripperType::PNEUMATIC_2FINGER: {
            // Pneumatic: Set output, wait for sensor
            io_.setOutput(IOChannel::GRIPPER_CLOSE, true);
            io_.setOutput(IOChannel::GRIPPER_OPEN, false);

            if (!waitForState(GripperState::CLOSED, timeout_ms)) {
                io_.setOutput(IOChannel::GRIPPER_CLOSE, false);
                status_.state = GripperState::ERROR;
                return false;
            }

            // Settle time
            std::this_thread::sleep_for(
                std::chrono::milliseconds(config_.settle_time_ms));

            // Check part detected
            updateStatus();
            if (!status_.part_detected) {
                // Không có chi tiết - mở lại
                release();
                return false;
            }

            status_.state = GripperState::GRIPPING;
            return true;
        }

        case GripperType::PNEUMATIC_VACUUM: {
            enableVacuum();

            // Wait for vacuum level
            auto start = std::chrono::steady_clock::now();
            while (true) {
                updateStatus();
                if (status_.vacuum_kpa <= config_.vacuum_threshold_kpa) {
                    status_.state = GripperState::GRIPPING;
                    return true;
                }

                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - start).count();
                if (elapsed > config_.vacuum_timeout_ms) {
                    disableVacuum();
                    status_.state = GripperState::ERROR;
                    return false;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }

        case GripperType::ELECTRIC_SERVO:
            // TODO: Implement Modbus communication
            return false;
    }

    return false;
}

bool GripperController::release(uint32_t timeout_ms) {
    if (timeout_ms == 0) {
        timeout_ms = config_.open_timeout_ms;
    }

    switch (config_.type) {
        case GripperType::PNEUMATIC_2FINGER: {
            io_.setOutput(IOChannel::GRIPPER_OPEN, true);
            io_.setOutput(IOChannel::GRIPPER_CLOSE, false);

            if (!waitForState(GripperState::OPENED, timeout_ms)) {
                status_.state = GripperState::ERROR;
                return false;
            }

            std::this_thread::sleep_for(
                std::chrono::milliseconds(config_.settle_time_ms));

            io_.setOutput(IOChannel::GRIPPER_OPEN, false);
            status_.state = GripperState::OPENED;
            return true;
        }

        case GripperType::PNEUMATIC_VACUUM: {
            disableVacuum();
            blowOff();
            std::this_thread::sleep_for(
                std::chrono::milliseconds(config_.blowoff_time_ms));
            io_.setOutput(IOChannel::BLOW_OFF, false);
            status_.state = GripperState::OPENED;
            return true;
        }

        case GripperType::ELECTRIC_SERVO:
            return false;
    }

    return false;
}

void GripperController::updateStatus() {
    status_.sensor_closed = io_.getInput(IOChannel::GRIPPER_CLOSED);
    status_.sensor_opened = io_.getInput(IOChannel::GRIPPER_OPENED);
    status_.part_detected = io_.getInput(IOChannel::PART_DETECTED);
    status_.vacuum_kpa = io_.getAnalogInput(IOChannel::VACUUM_PRESSURE) * -100.0;
    status_.force_n = io_.getAnalogInput(IOChannel::GRIPPER_FORCE) * 100.0;

    // Determine state from sensors
    if (status_.sensor_closed && !status_.sensor_opened) {
        status_.state = status_.part_detected ?
            GripperState::GRIPPING : GripperState::CLOSED;
    } else if (!status_.sensor_closed && status_.sensor_opened) {
        status_.state = GripperState::OPENED;
    } else if (status_.sensor_closed && status_.sensor_opened) {
        status_.state = GripperState::ERROR;  // Both sensors - error
    }
}

} // namespace Motion
```

---

## 3. Vision System Integration

### 3.1 Camera Selection

| Feature | 2D Camera | 3D Camera |
|---------|-----------|-----------|
| **Hardware** | Basler ace, FLIR | RealSense D435, Zivid |
| **Output** | Image + 2D pose | Point cloud + 6D pose |
| **Accuracy** | ±0.5mm (với calib) | ±0.1mm |
| **FOV** | Rộng | Hẹp hơn |
| **Chi phí** | Thấp | Cao |
| **Use case** | Flat parts, 2.5D | Complex shapes, bin picking |

### 3.2 Parallax Compensation (2D Camera)

Khi dùng 2D camera, cần bù parallax error do khoảng cách Z thay đổi:

```
                Camera
                   │
                   │ Z_ref (khoảng cách chuẩn)
                   ▼
    ┌──────────────────────────┐
    │      Reference Plane     │  ← Calibration plane
    └──────────────────────────┘
                   │
                   │ ΔZ (offset)
                   ▼
    ┌──────────────────────────┐
    │      Object Plane        │  ← Actual object
    └──────────────────────────┘

    Parallax error = pixel_offset × (ΔZ / Z_ref)
```

### 3.3 ZeroMQ Vision Bridge

```cpp
// VisionBridge.h
#pragma once
#include <zmq.hpp>
#include <nlohmann/json.hpp>
#include <optional>

namespace Vision {

struct DetectedObject {
    int id;
    std::string class_name;
    double confidence;

    // Pose in camera frame
    double x, y, z;           // Position (mm)
    double rx, ry, rz;        // Rotation (deg)

    // Bounding box (pixels)
    int bbox_x, bbox_y, bbox_w, bbox_h;
};

struct VisionResult {
    uint64_t timestamp_us;
    int frame_id;
    std::vector<DetectedObject> objects;
    bool success;
    std::string error_message;
};

class VisionBridge {
public:
    VisionBridge();
    ~VisionBridge();

    // Connection
    bool connect(const std::string& address = "tcp://127.0.0.1:5570");
    void disconnect();
    bool isConnected() const;

    // Commands
    bool triggerCapture();
    std::optional<VisionResult> getLastResult(uint32_t timeout_ms = 1000);

    // Convenience
    std::optional<DetectedObject> detectSingleObject(
        const std::string& class_filter = "",
        uint32_t timeout_ms = 2000);

    // Calibration
    bool startCalibration();
    bool captureCalibrationImage();
    bool computeCalibration();
    std::array<double, 16> getHandEyeMatrix() const;

private:
    zmq::context_t context_;
    zmq::socket_t socket_;
    bool connected_ = false;
    std::array<double, 16> hand_eye_matrix_;

    nlohmann::json sendCommand(const nlohmann::json& cmd, uint32_t timeout_ms);
};

} // namespace Vision
```

### 3.4 Vision Protocol (JSON over ZeroMQ)

```json
// Request: Trigger capture
{
    "command": "capture",
    "params": {
        "exposure_ms": 10,
        "gain_db": 0
    }
}

// Response: Detection result
{
    "success": true,
    "timestamp_us": 1234567890,
    "frame_id": 42,
    "objects": [
        {
            "id": 1,
            "class": "bolt_m8",
            "confidence": 0.95,
            "pose": {
                "x": 150.5,
                "y": -30.2,
                "z": 5.0,
                "rx": 0.0,
                "ry": 0.0,
                "rz": 45.3
            },
            "bbox": [100, 200, 50, 50]
        }
    ]
}
```

### 3.5 Python Vision Process

```python
# vision_server.py
import zmq
import cv2
import numpy as np
from dataclasses import dataclass
from typing import List, Optional
import json

@dataclass
class DetectedObject:
    id: int
    class_name: str
    confidence: float
    x: float
    y: float
    z: float
    rx: float = 0.0
    ry: float = 0.0
    rz: float = 0.0

class VisionServer:
    def __init__(self, camera_id: int = 0):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        self.camera = cv2.VideoCapture(camera_id)

        # Hand-eye calibration matrix (4x4)
        self.T_cam_to_flange = np.eye(4)

        # Camera intrinsics
        self.camera_matrix = None
        self.dist_coeffs = None

    def start(self, port: int = 5570):
        self.socket.bind(f"tcp://*:{port}")
        print(f"Vision server listening on port {port}")

        while True:
            message = self.socket.recv_json()
            response = self.handle_command(message)
            self.socket.send_json(response)

    def handle_command(self, cmd: dict) -> dict:
        command = cmd.get("command", "")

        if command == "capture":
            return self.capture_and_detect(cmd.get("params", {}))
        elif command == "calibrate_start":
            return self.start_calibration()
        elif command == "calibrate_capture":
            return self.capture_calibration_image()
        elif command == "calibrate_compute":
            return self.compute_calibration()
        else:
            return {"success": False, "error": f"Unknown command: {command}"}

    def capture_and_detect(self, params: dict) -> dict:
        ret, frame = self.camera.read()
        if not ret:
            return {"success": False, "error": "Failed to capture image"}

        # TODO: Run detection model (YOLO, etc.)
        objects = self.detect_objects(frame)

        return {
            "success": True,
            "timestamp_us": int(time.time() * 1e6),
            "frame_id": self.frame_count,
            "objects": [self.object_to_dict(obj) for obj in objects]
        }

    def detect_objects(self, frame: np.ndarray) -> List[DetectedObject]:
        # Placeholder - implement actual detection
        # Using ArUco markers for demo
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters_create()

        corners, ids, rejected = cv2.aruco.detectMarkers(
            frame, aruco_dict, parameters=parameters)

        objects = []
        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, 0.05, self.camera_matrix, self.dist_coeffs)

            for i, marker_id in enumerate(ids.flatten()):
                t = tvecs[i][0] * 1000  # Convert to mm
                r = cv2.Rodrigues(rvecs[i])[0]
                euler = self.rotation_matrix_to_euler(r)

                objects.append(DetectedObject(
                    id=int(marker_id),
                    class_name=f"marker_{marker_id}",
                    confidence=1.0,
                    x=t[0], y=t[1], z=t[2],
                    rx=euler[0], ry=euler[1], rz=euler[2]
                ))

        return objects

if __name__ == "__main__":
    server = VisionServer(camera_id=0)
    server.start(port=5570)
```

---

## 4. Hand-Eye Calibration

### 4.1 Eye-in-Hand vs Eye-to-Hand

| Configuration | Camera Mount | Use Case |
|---------------|--------------|----------|
| **Eye-in-Hand** | Trên flange robot | Bin picking, flexible |
| **Eye-to-Hand** | Cố định trên frame | Conveyor tracking, lớn |

### 4.2 Phương trình AX = XB

```
Eye-in-Hand:
┌─────┐       ┌─────┐       ┌─────┐
│Robot│ ──A── │Flange│ ──X── │Camera│
│Base │       └─────┘       └──┬──┘
└─────┘                        │ B
                               ▼
                          ┌────────┐
                          │ Target │
                          └────────┘

A = T_base_to_flange (từ robot FK)
B = T_camera_to_target (từ vision)
X = T_flange_to_camera (cần tìm)

Phương trình: A₁ × X × B₁ = A₂ × X × B₂

Biến đổi: (A₂⁻¹ × A₁) × X = X × (B₂ × B₁⁻¹)
                ↓                    ↓
                A'                   B'

Giải AX = XB để tìm X
```

### 4.3 Quy trình Calibration

```cpp
// HandEyeCalibrator.h
#pragma once
#include <vector>
#include <array>
#include <Eigen/Dense>

namespace Vision {

struct CalibrationPose {
    // Robot pose (from FK)
    std::array<double, 6> joint_angles;  // J1-J6
    Eigen::Matrix4d T_base_to_flange;

    // Vision pose (from detection)
    Eigen::Matrix4d T_camera_to_target;

    bool valid = false;
};

class HandEyeCalibrator {
public:
    // Clear previous data
    void reset();

    // Add calibration pose pair
    bool addPose(const CalibrationPose& pose);

    // Check if enough poses collected
    bool hasEnoughPoses() const { return poses_.size() >= 10; }
    size_t getPoseCount() const { return poses_.size(); }

    // Compute calibration
    bool compute();

    // Get result
    Eigen::Matrix4d getHandEyeMatrix() const { return T_hand_eye_; }
    double getReprojectionError() const { return reproj_error_; }

    // Validate result
    bool validate(const CalibrationPose& test_pose, double tolerance_mm = 1.0);

private:
    std::vector<CalibrationPose> poses_;
    Eigen::Matrix4d T_hand_eye_;
    double reproj_error_ = 0.0;

    // Tsai-Lenz method
    bool solveTsaiLenz();

    // OpenCV wrapper
    bool solveOpenCV();
};

} // namespace Vision
```

### 4.4 Calibration Procedure (HMI)

```
┌─────────────────────────────────────────────────────────────┐
│                  HAND-EYE CALIBRATION                       │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Step 1: Place calibration target (checkerboard/ArUco)     │
│  Step 2: Move robot to capture poses (min 10-15 poses)     │
│  Step 3: Compute calibration matrix                        │
│  Step 4: Validate with test poses                          │
│                                                             │
│  ┌─────────────────────┐  ┌────────────────────────────┐   │
│  │                     │  │ Pose List:                 │   │
│  │   Camera Preview    │  │ ┌─────┬────────┬────────┐  │   │
│  │                     │  │ │ # │ Robot  │ Vision │  │   │
│  │   [Detected: ✓]     │  │ ├─────┼────────┼────────┤  │   │
│  │                     │  │ │ 1  │ ✓      │ ✓      │  │   │
│  │                     │  │ │ 2  │ ✓      │ ✓      │  │   │
│  └─────────────────────┘  │ │ 3  │ ✓      │ ✓      │  │   │
│                           │ │ ...│        │        │  │   │
│  Robot Position:          │ └─────┴────────┴────────┘  │   │
│  J1: 45.2°  J4: -30.1°    │                            │   │
│  J2: -60.5° J5: 90.0°     │ Poses captured: 8/15       │   │
│  J3: 120.3° J6: 0.0°      │                            │   │
│                           └────────────────────────────┘   │
│                                                             │
│  [Capture Pose]  [Delete Last]  [Compute]  [Validate]      │
│                                                             │
│  Status: Move robot to different orientations              │
│  Reprojection Error: -- mm                                 │
└─────────────────────────────────────────────────────────────┘
```

---

## 5. Pick & Place Cycle FSM

### 5.1 State Machine

```
                    ┌─────────┐
                    │  IDLE   │◄────────────────────┐
                    └────┬────┘                     │
                         │ Start cycle              │
                         ▼                          │
                ┌────────────────┐                  │
                │ MOVE_TO_SCAN  │                  │
                └───────┬────────┘                  │
                        │ At position              │
                        ▼                          │
                ┌────────────────┐                  │
                │    SCANNING   │───── No object ──┤
                └───────┬────────┘      found      │
                        │ Object detected          │
                        ▼                          │
                ┌────────────────┐                  │
                │ MOVE_TO_PICK  │                  │
                └───────┬────────┘                  │
                        │ At pick position         │
                        ▼                          │
                ┌────────────────┐                  │
                │ APPROACH_PICK │                  │
                └───────┬────────┘                  │
                        │ At object                │
                        ▼                          │
                ┌────────────────┐                  │
         ┌──────│   GRIPPING    │                  │
         │      └───────┬────────┘                  │
    Grip │              │ Part gripped             │
    fail │              ▼                          │
         │      ┌────────────────┐                  │
         │      │  RETRACT_PICK │                  │
         │      └───────┬────────┘                  │
         │              │                          │
         │              ▼                          │
         │      ┌────────────────┐                  │
         │      │ MOVE_TO_PLACE │                  │
         │      └───────┬────────┘                  │
         │              │ At place position        │
         │              ▼                          │
         │      ┌────────────────┐                  │
         │      │APPROACH_PLACE │                  │
         │      └───────┬────────┘                  │
         │              │                          │
         │              ▼                          │
         │      ┌────────────────┐                  │
         │      │   RELEASING   │                  │
         │      └───────┬────────┘                  │
         │              │ Part released            │
         │              ▼                          │
         │      ┌────────────────┐                  │
         │      │ RETRACT_PLACE │                  │
         │      └───────┬────────┘                  │
         │              │                          │
         └──────────────┴──────────────────────────┘
```

### 5.2 Pick & Place Manager

```cpp
// PickPlaceManager.h
#pragma once
#include "GripperController.h"
#include "VisionBridge.h"
#include "../Trajectory/TrajectoryPlanner.h"

namespace Motion {

enum class PickPlaceState {
    IDLE,
    MOVE_TO_SCAN,
    SCANNING,
    MOVE_TO_PICK,
    APPROACH_PICK,
    GRIPPING,
    RETRACT_PICK,
    MOVE_TO_PLACE,
    APPROACH_PLACE,
    RELEASING,
    RETRACT_PLACE,
    ERROR,
    PAUSED
};

struct PickPlaceConfig {
    // Positions (mm)
    CartesianPose scan_position;
    CartesianPose place_position;

    // Approach/retract distances
    double approach_distance_mm = 50.0;
    double retract_distance_mm = 50.0;

    // Speeds
    double move_speed_mm_s = 500.0;
    double approach_speed_mm_s = 100.0;

    // Vision
    std::string object_class_filter;  // Empty = any object
    double min_confidence = 0.8;

    // Retry
    int max_grip_retries = 3;
    int max_scan_retries = 5;
};

struct PickPlaceStatus {
    PickPlaceState state = PickPlaceState::IDLE;
    int cycle_count = 0;
    int success_count = 0;
    int fail_count = 0;

    std::optional<Vision::DetectedObject> current_target;
    std::string last_error;
};

class PickPlaceManager {
public:
    PickPlaceManager(
        TrajectoryPlanner& trajectory,
        GripperController& gripper,
        Vision::VisionBridge& vision);

    // Configuration
    void configure(const PickPlaceConfig& config);

    // Control
    void start();
    void stop();
    void pause();
    void resume();

    // Single cycle
    bool runSingleCycle();

    // Continuous operation
    void runContinuous(int max_cycles = -1);  // -1 = infinite

    // Status
    PickPlaceStatus getStatus() const;
    bool isRunning() const;

    // Callbacks
    using CycleCallback = std::function<void(int cycle, bool success)>;
    void setCycleCallback(CycleCallback cb) { cycle_callback_ = cb; }

private:
    void stateMachine();
    bool transitionTo(PickPlaceState new_state);

    // State handlers
    void handleIdle();
    void handleMoveToScan();
    void handleScanning();
    void handleMoveToPick();
    void handleApproachPick();
    void handleGripping();
    void handleRetractPick();
    void handleMoveToPlace();
    void handleApproachPlace();
    void handleReleasing();
    void handleRetractPlace();
    void handleError();

    // Helpers
    CartesianPose computePickPose(const Vision::DetectedObject& obj);
    CartesianPose applyApproachOffset(const CartesianPose& pose, double offset);

    TrajectoryPlanner& trajectory_;
    GripperController& gripper_;
    Vision::VisionBridge& vision_;

    PickPlaceConfig config_;
    PickPlaceStatus status_;

    std::atomic<bool> running_{false};
    std::atomic<bool> paused_{false};

    int grip_retry_count_ = 0;
    int scan_retry_count_ = 0;

    CycleCallback cycle_callback_;
};

} // namespace Motion
```

### 5.3 Implementation

```cpp
// PickPlaceManager.cpp
#include "PickPlaceManager.h"
#include <spdlog/spdlog.h>

namespace Motion {

bool PickPlaceManager::runSingleCycle() {
    status_.cycle_count++;
    running_ = true;

    while (running_ && status_.state != PickPlaceState::IDLE) {
        if (paused_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        stateMachine();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    bool success = (status_.state == PickPlaceState::IDLE &&
                   status_.last_error.empty());

    if (success) {
        status_.success_count++;
    } else {
        status_.fail_count++;
    }

    if (cycle_callback_) {
        cycle_callback_(status_.cycle_count, success);
    }

    return success;
}

void PickPlaceManager::handleScanning() {
    auto result = vision_.detectSingleObject(
        config_.object_class_filter, 2000);

    if (!result.has_value()) {
        scan_retry_count_++;
        if (scan_retry_count_ >= config_.max_scan_retries) {
            status_.last_error = "No object found after max retries";
            transitionTo(PickPlaceState::IDLE);
        }
        return;
    }

    if (result->confidence < config_.min_confidence) {
        spdlog::warn("Object confidence {:.2f} below threshold {:.2f}",
                    result->confidence, config_.min_confidence);
        return;
    }

    status_.current_target = result;
    scan_retry_count_ = 0;
    transitionTo(PickPlaceState::MOVE_TO_PICK);
}

void PickPlaceManager::handleMoveToPick() {
    if (!status_.current_target.has_value()) {
        transitionTo(PickPlaceState::ERROR);
        return;
    }

    CartesianPose pick_pose = computePickPose(*status_.current_target);
    CartesianPose approach_pose = applyApproachOffset(
        pick_pose, -config_.approach_distance_mm);

    if (!trajectory_.moveL(approach_pose, config_.move_speed_mm_s)) {
        status_.last_error = "Failed to move to pick approach";
        transitionTo(PickPlaceState::ERROR);
        return;
    }

    if (trajectory_.isMotionComplete()) {
        transitionTo(PickPlaceState::APPROACH_PICK);
    }
}

void PickPlaceManager::handleGripping() {
    if (!gripper_.grip(config_.gripper_timeout_ms)) {
        grip_retry_count_++;
        if (grip_retry_count_ >= config_.max_grip_retries) {
            status_.last_error = "Grip failed after max retries";
            transitionTo(PickPlaceState::ERROR);
        } else {
            // Retry - back to approach
            transitionTo(PickPlaceState::APPROACH_PICK);
        }
        return;
    }

    if (!gripper_.hasPart()) {
        status_.last_error = "No part detected after grip";
        transitionTo(PickPlaceState::ERROR);
        return;
    }

    grip_retry_count_ = 0;
    transitionTo(PickPlaceState::RETRACT_PICK);
}

CartesianPose PickPlaceManager::computePickPose(
    const Vision::DetectedObject& obj) {

    // Transform from camera frame to base frame
    // P_base = T_base_flange × T_flange_camera × P_camera

    Eigen::Matrix4d T_base_flange = trajectory_.getCurrentTransform();
    Eigen::Matrix4d T_flange_camera = vision_.getHandEyeMatrix();

    Eigen::Vector4d P_camera(obj.x, obj.y, obj.z, 1.0);
    Eigen::Vector4d P_base = T_base_flange * T_flange_camera * P_camera;

    CartesianPose pick_pose;
    pick_pose.x = P_base(0);
    pick_pose.y = P_base(1);
    pick_pose.z = P_base(2);

    // Orientation from detection
    pick_pose.rx = obj.rx;
    pick_pose.ry = obj.ry;
    pick_pose.rz = obj.rz;

    return pick_pose;
}

} // namespace Motion
```

---

## 6. Palletizing Patterns

### 6.1 Pattern Types

```
┌─────────────────────────────────────────────────────────────────┐
│                     PALLETIZING PATTERNS                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Snake Pattern:              Zigzag Pattern:                    │
│  ┌───┬───┬───┬───┐          ┌───┬───┬───┬───┐                  │
│  │ 1 │ 2 │ 3 │ 4 │          │ 1 │ 2 │ 3 │ 4 │                  │
│  ├───┼───┼───┼───┤          ├───┼───┼───┼───┤                  │
│  │ 8 │ 7 │ 6 │ 5 │          │ 8 │ 7 │ 6 │ 5 │                  │
│  ├───┼───┼───┼───┤          ├───┼───┼───┼───┤                  │
│  │ 9 │10 │11 │12 │          │ 9 │10 │11 │12 │                  │
│  └───┴───┴───┴───┘          └───┴───┴───┴───┘                  │
│                                                                 │
│  Row-by-Row Pattern:         Spiral Pattern:                    │
│  ┌───┬───┬───┬───┐          ┌───┬───┬───┬───┐                  │
│  │ 1 │ 2 │ 3 │ 4 │          │ 1 │ 2 │ 3 │ 4 │                  │
│  ├───┼───┼───┼───┤          ├───┼───┼───┼───┤                  │
│  │ 5 │ 6 │ 7 │ 8 │          │12 │13 │14 │ 5 │                  │
│  ├───┼───┼───┼───┤          ├───┼───┼───┼───┤                  │
│  │ 9 │10 │11 │12 │          │11 │16 │15 │ 6 │                  │
│  └───┴───┴───┴───┘          ├───┼───┼───┼───┤                  │
│                              │10 │ 9 │ 8 │ 7 │                  │
│                              └───┴───┴───┴───┘                  │
└─────────────────────────────────────────────────────────────────┘
```

### 6.2 Pallet Configuration

```cpp
// PalletPattern.h
#pragma once
#include <vector>

namespace Motion {

enum class PatternType {
    SNAKE,
    ZIGZAG,
    ROW_BY_ROW,
    COLUMN_BY_COLUMN,
    SPIRAL
};

struct PalletConfig {
    // Pallet dimensions
    int rows = 4;
    int columns = 4;
    int layers = 3;

    // Spacing (mm)
    double pitch_x = 100.0;
    double pitch_y = 100.0;
    double layer_height = 50.0;

    // Reference point (corner 1)
    CartesianPose origin;

    // Second corner (for rotation calculation)
    CartesianPose corner2;

    // Pattern
    PatternType pattern = PatternType::SNAKE;
    bool start_bottom_left = true;
};

class PalletPattern {
public:
    explicit PalletPattern(const PalletConfig& config);

    // Get position for index
    CartesianPose getPosition(int index) const;

    // Get next position (auto-increment)
    CartesianPose getNextPosition();

    // Current state
    int getCurrentIndex() const { return current_index_; }
    int getCurrentRow() const { return current_row_; }
    int getCurrentColumn() const { return current_col_; }
    int getCurrentLayer() const { return current_layer_; }

    // Capacity
    int getTotalPositions() const;
    bool isFull() const { return current_index_ >= getTotalPositions(); }
    void reset() { current_index_ = 0; }

    // Manual control
    void setLayer(int layer);
    void skipToPosition(int index);

private:
    std::pair<int, int> indexToRowCol(int index) const;
    CartesianPose computePose(int row, int col, int layer) const;

    PalletConfig config_;
    int current_index_ = 0;
    int current_row_ = 0;
    int current_col_ = 0;
    int current_layer_ = 0;

    // Computed from corner points
    Eigen::Matrix3d rotation_;
    Eigen::Vector3d x_axis_;
    Eigen::Vector3d y_axis_;
};

} // namespace Motion
```

### 6.3 Pattern Generation

```cpp
// PalletPattern.cpp
#include "PalletPattern.h"

namespace Motion {

std::pair<int, int> PalletPattern::indexToRowCol(int index) const {
    int positions_per_layer = config_.rows * config_.columns;
    int layer_index = index % positions_per_layer;

    int row, col;

    switch (config_.pattern) {
        case PatternType::SNAKE: {
            row = layer_index / config_.columns;
            int col_in_row = layer_index % config_.columns;
            col = (row % 2 == 0) ? col_in_row : (config_.columns - 1 - col_in_row);
            break;
        }

        case PatternType::ROW_BY_ROW: {
            row = layer_index / config_.columns;
            col = layer_index % config_.columns;
            break;
        }

        case PatternType::COLUMN_BY_COLUMN: {
            col = layer_index / config_.rows;
            row = layer_index % config_.rows;
            break;
        }

        case PatternType::SPIRAL: {
            // Spiral from outside to inside
            auto [r, c] = computeSpiralPosition(layer_index,
                                                config_.rows,
                                                config_.columns);
            row = r;
            col = c;
            break;
        }

        default:
            row = layer_index / config_.columns;
            col = layer_index % config_.columns;
    }

    // Flip if not starting bottom-left
    if (!config_.start_bottom_left) {
        row = config_.rows - 1 - row;
    }

    return {row, col};
}

CartesianPose PalletPattern::computePose(int row, int col, int layer) const {
    // Compute offset from origin
    Eigen::Vector3d offset;
    offset(0) = col * config_.pitch_x;
    offset(1) = row * config_.pitch_y;
    offset(2) = layer * config_.layer_height;

    // Apply pallet rotation
    Eigen::Vector3d world_offset = rotation_ * offset;

    CartesianPose pose = config_.origin;
    pose.x += world_offset(0);
    pose.y += world_offset(1);
    pose.z += world_offset(2);

    return pose;
}

} // namespace Motion
```

---

## 7. Tool Change

### 7.1 Tool Length Offset (TLO)

```
        ┌─────────────┐
        │   Flange    │
        ├─────────────┤ ← Z = 0 (flange face)
        │             │
        │  Welding    │ TLO = -150mm
        │   Torch     │
        │             │
        └──────┬──────┘
               ▼ TCP

        ┌─────────────┐
        │   Flange    │
        ├─────────────┤ ← Z = 0
        │             │
        │  Gripper    │ TLO = -80mm
        │             │
        └──────┬──────┘
               ▼ TCP
```

### 7.2 Tool Management

```cpp
// ToolManager.h
#pragma once
#include <map>
#include <string>

namespace Motion {

struct ToolDefinition {
    int tool_number;      // T1, T2, etc.
    std::string name;

    // Geometry (mm)
    double length_z;      // TLO
    double offset_x = 0;
    double offset_y = 0;

    // Orientation offset (deg)
    double rx = 0;
    double ry = 0;
    double rz = 0;

    // Mass (kg) - for dynamics
    double mass = 0;

    // Tool-specific I/O
    std::vector<int> associated_outputs;
};

class ToolManager {
public:
    // Tool definitions
    void defineTool(const ToolDefinition& tool);
    const ToolDefinition& getTool(int tool_number) const;

    // Active tool
    int getActiveTool() const { return active_tool_; }
    bool setActiveTool(int tool_number);

    // G-code generation for grblHAL
    std::string generateToolChangeGcode(int new_tool) const;

    // Apply TLO to position
    CartesianPose applyTLO(const CartesianPose& pose, int tool_number) const;
    CartesianPose removeTLO(const CartesianPose& pose, int tool_number) const;

private:
    std::map<int, ToolDefinition> tools_;
    int active_tool_ = 0;  // 0 = no tool
};

} // namespace Motion
```

### 7.3 Tool Change Sequence

```cpp
std::string ToolManager::generateToolChangeGcode(int new_tool) const {
    std::ostringstream gcode;

    // Cancel current TLO
    gcode << "G49\n";  // Cancel tool length compensation

    // Select new tool
    gcode << "T" << new_tool << "\n";

    // Apply new TLO if tool defined
    if (tools_.count(new_tool)) {
        const auto& tool = tools_.at(new_tool);
        // G43 H<tool> - Apply tool length offset from tool table
        gcode << "G43 H" << new_tool << "\n";

        // Or explicit offset: G43.1 Z<offset>
        // gcode << "G43.1 Z" << std::fixed << std::setprecision(3)
        //       << tool.length_z << "\n";
    }

    return gcode.str();
}
```

---

## 8. Mode Switching

### 8.1 FSM chuyển đổi Mode

```
                         WELDING MODE
                              │
                              │ Request mode change
                              ▼
                    ┌─────────────────┐
                    │   STOP_WELD     │
                    │ (Postflow done) │
                    └────────┬────────┘
                             │
                             ▼
                    ┌─────────────────┐
                    │  PARK_TORCH     │
                    │ (Safe position) │
                    └────────┬────────┘
                             │
                             ▼
                    ┌─────────────────┐
                    │  TOOL_CHANGE    │◄──── Automatic or manual
                    │ (Mount gripper) │
                    └────────┬────────┘
                             │
                             ▼
                    ┌─────────────────┐
                    │  UPDATE_TLO     │
                    │ (New tool data) │
                    └────────┬────────┘
                             │
                             ▼
                    ┌─────────────────┐
                    │ CALIBRATE_TOOL  │◄──── Optional
                    │ (Touch sensing) │
                    └────────┬────────┘
                             │
                             ▼
                       PICKPLACE MODE
```

### 8.2 Mode Manager

```cpp
// ModeManager.h
#pragma once
#include "WeldingSequencer.h"
#include "PickPlaceManager.h"
#include "ToolManager.h"

namespace Motion {

enum class OperationMode {
    WELDING,
    PICKPLACE,
    SCAN_TO_PATH,
    MANUAL,
    CHANGEOVER  // During mode switch
};

class ModeManager {
public:
    ModeManager(
        WeldingSequencer& welding,
        PickPlaceManager& pickplace,
        ToolManager& tools);

    // Current mode
    OperationMode getCurrentMode() const { return current_mode_; }

    // Request mode change (async)
    bool requestModeChange(OperationMode new_mode);

    // Check if mode change is safe
    bool canChangeMode() const;

    // Mode change status
    bool isChangingMode() const { return current_mode_ == OperationMode::CHANGEOVER; }
    float getChangeProgress() const { return change_progress_; }

    // Callbacks
    using ModeChangedCallback = std::function<void(OperationMode old, OperationMode new_mode)>;
    void setModeChangedCallback(ModeChangedCallback cb) { mode_changed_cb_ = cb; }

private:
    void executeChangeover(OperationMode target);
    bool parkCurrentTool();
    bool changeTool(int new_tool);
    bool calibrateTool();

    WeldingSequencer& welding_;
    PickPlaceManager& pickplace_;
    ToolManager& tools_;

    OperationMode current_mode_ = OperationMode::MANUAL;
    OperationMode target_mode_;
    float change_progress_ = 0.0f;

    ModeChangedCallback mode_changed_cb_;
};

} // namespace Motion
```

---

## 9. Safety Considerations

### 9.1 Pick & Place Specific Hazards

| Hazard | Mitigation |
|--------|------------|
| **Part drop** | Vacuum/grip sensor, catch tray |
| **Collision** | Vision-based obstacle detection |
| **Pinch point** | Force limiting on gripper |
| **Vision failure** | Timeout, fallback position |

### 9.2 Safety Interlocks

```cpp
struct PickPlaceSafetyConfig {
    // Gripper safety
    double max_grip_force_n = 100.0;      // Force limit
    uint32_t grip_monitor_interval_ms = 50;
    bool require_part_sensor = true;

    // Vision safety
    uint32_t vision_timeout_ms = 5000;    // Max wait for detection
    double min_detection_confidence = 0.7;

    // Motion safety
    double max_approach_speed_mm_s = 200.0;
    double safe_retract_height_mm = 100.0;

    // Zone restrictions
    bool enable_pick_zone = true;
    BoundingBox pick_zone;
    bool enable_place_zone = true;
    BoundingBox place_zone;
};

class PickPlaceSafety {
public:
    // Check before motion
    bool validatePickPosition(const CartesianPose& pose);
    bool validatePlacePosition(const CartesianPose& pose);

    // Continuous monitoring
    void monitorGripForce();
    void checkPartPresence();

    // Emergency response
    void emergencyRelease();  // Open gripper immediately
    void emergencyRetract();  // Move to safe height
};
```

### 9.3 Vision Watchdog

```cpp
class VisionWatchdog {
public:
    void start(uint32_t timeout_ms);
    void feed();  // Call when valid detection received
    bool isExpired() const;

    // Action on timeout
    void setTimeoutAction(std::function<void()> action);

private:
    std::chrono::steady_clock::time_point last_feed_;
    uint32_t timeout_ms_;
    std::function<void()> timeout_action_;
};

// Usage in PickPlaceManager
void PickPlaceManager::handleScanning() {
    vision_watchdog_.start(config_.vision_timeout_ms);

    while (!vision_watchdog_.isExpired()) {
        auto result = vision_.detectSingleObject();
        if (result.has_value()) {
            vision_watchdog_.feed();
            // Process result...
        }
    }

    // Timeout - go to safe state
    transitionTo(PickPlaceState::ERROR);
}
```

---

## 10. HMI Integration

### 10.1 PickPlaceView

```xml
<!-- PickPlaceView.xaml -->
<UserControl x:Class="RobotHMI.Views.PickPlaceView">
    <Grid>
        <Grid.ColumnDefinitions>
            <ColumnDefinition Width="2*"/>
            <ColumnDefinition Width="1*"/>
        </Grid.ColumnDefinitions>

        <!-- Camera/Vision Display -->
        <Border Grid.Column="0" Background="#1E1E1E">
            <Grid>
                <Image Source="{Binding CameraImage}" Stretch="Uniform"/>

                <!-- Detection overlay -->
                <ItemsControl ItemsSource="{Binding DetectedObjects}">
                    <ItemsControl.ItemTemplate>
                        <DataTemplate>
                            <Canvas>
                                <Rectangle
                                    Canvas.Left="{Binding BboxX}"
                                    Canvas.Top="{Binding BboxY}"
                                    Width="{Binding BboxW}"
                                    Height="{Binding BboxH}"
                                    Stroke="Lime" StrokeThickness="2"/>
                                <TextBlock
                                    Canvas.Left="{Binding BboxX}"
                                    Canvas.Top="{Binding LabelY}"
                                    Text="{Binding Label}"
                                    Foreground="Lime"/>
                            </Canvas>
                        </DataTemplate>
                    </ItemsControl.ItemTemplate>
                </ItemsControl>
            </Grid>
        </Border>

        <!-- Control Panel -->
        <StackPanel Grid.Column="1" Margin="10">
            <!-- State Display -->
            <GroupBox Header="Status">
                <StackPanel>
                    <TextBlock Text="{Binding CurrentState}"
                               FontSize="18" FontWeight="Bold"/>
                    <TextBlock Text="{Binding CycleInfo}"/>
                    <ProgressBar Value="{Binding CycleProgress}"
                                 Height="20" Margin="0,10"/>
                </StackPanel>
            </GroupBox>

            <!-- Statistics -->
            <GroupBox Header="Statistics">
                <Grid>
                    <Grid.ColumnDefinitions>
                        <ColumnDefinition/>
                        <ColumnDefinition/>
                    </Grid.ColumnDefinitions>
                    <StackPanel Grid.Column="0">
                        <TextBlock Text="Cycles:"/>
                        <TextBlock Text="Success:"/>
                        <TextBlock Text="Failed:"/>
                    </StackPanel>
                    <StackPanel Grid.Column="1">
                        <TextBlock Text="{Binding CycleCount}"/>
                        <TextBlock Text="{Binding SuccessCount}" Foreground="Green"/>
                        <TextBlock Text="{Binding FailCount}" Foreground="Red"/>
                    </StackPanel>
                </Grid>
            </GroupBox>

            <!-- Gripper Status -->
            <GroupBox Header="Gripper">
                <StackPanel>
                    <TextBlock Text="{Binding GripperState}"/>
                    <StackPanel Orientation="Horizontal">
                        <Ellipse Width="12" Height="12"
                                 Fill="{Binding PartDetectedColor}"/>
                        <TextBlock Text="Part Detected" Margin="5,0"/>
                    </StackPanel>
                </StackPanel>
            </GroupBox>

            <!-- Controls -->
            <StackPanel Orientation="Horizontal" Margin="0,20">
                <Button Content="START" Command="{Binding StartCommand}"
                        Width="80" Height="40" Margin="5"/>
                <Button Content="STOP" Command="{Binding StopCommand}"
                        Width="80" Height="40" Margin="5"/>
                <Button Content="SINGLE" Command="{Binding SingleCycleCommand}"
                        Width="80" Height="40" Margin="5"/>
            </StackPanel>
        </StackPanel>
    </Grid>
</UserControl>
```

### 10.2 ViewModel

```csharp
// PickPlaceViewModel.cs
public partial class PickPlaceViewModel : ObservableObject
{
    private readonly IPickPlaceService _pickPlaceService;
    private readonly IVisionService _visionService;

    [ObservableProperty]
    private BitmapSource? _cameraImage;

    [ObservableProperty]
    private string _currentState = "IDLE";

    [ObservableProperty]
    private int _cycleCount;

    [ObservableProperty]
    private int _successCount;

    [ObservableProperty]
    private int _failCount;

    [ObservableProperty]
    private string _gripperState = "OPENED";

    [ObservableProperty]
    private bool _partDetected;

    public ObservableCollection<DetectedObjectVm> DetectedObjects { get; } = new();

    [RelayCommand]
    private async Task Start()
    {
        await _pickPlaceService.StartContinuousAsync();
    }

    [RelayCommand]
    private void Stop()
    {
        _pickPlaceService.Stop();
    }

    [RelayCommand]
    private async Task SingleCycle()
    {
        await _pickPlaceService.RunSingleCycleAsync();
    }

    private void OnVisionFrameReceived(VisionFrame frame)
    {
        CameraImage = frame.ToBitmapSource();

        DetectedObjects.Clear();
        foreach (var obj in frame.Objects)
        {
            DetectedObjects.Add(new DetectedObjectVm(obj));
        }
    }
}
```

---

## 11. Configuration Files

### 11.1 Pick & Place Config

```yaml
# config/pickplace.yaml
pickplace:
  # Scan position
  scan_position:
    x: 400.0
    y: 0.0
    z: 500.0
    rx: 180.0
    ry: 0.0
    rz: 0.0

  # Place position (pallet origin)
  place_position:
    x: 600.0
    y: -200.0
    z: 100.0
    rx: 180.0
    ry: 0.0
    rz: 0.0

  # Motion parameters
  approach_distance_mm: 50.0
  retract_distance_mm: 50.0
  move_speed_mm_s: 500.0
  approach_speed_mm_s: 100.0

  # Vision
  object_class_filter: ""  # Empty = any
  min_confidence: 0.8
  vision_timeout_ms: 5000

  # Retry
  max_grip_retries: 3
  max_scan_retries: 5

gripper:
  type: PNEUMATIC_2FINGER  # PNEUMATIC_2FINGER, PNEUMATIC_VACUUM, ELECTRIC_SERVO
  close_timeout_ms: 500
  open_timeout_ms: 500
  settle_time_ms: 50

pallet:
  rows: 4
  columns: 4
  layers: 2
  pitch_x: 100.0
  pitch_y: 100.0
  layer_height: 50.0
  pattern: SNAKE  # SNAKE, ZIGZAG, ROW_BY_ROW, SPIRAL

vision:
  camera_address: "tcp://127.0.0.1:5570"
  hand_eye_calibration_file: "config/hand_eye_calib.yaml"
```

### 11.2 Hand-Eye Calibration File

```yaml
# config/hand_eye_calib.yaml
hand_eye_calibration:
  method: "tsai_lenz"
  date: "2026-01-15"

  # T_flange_to_camera (4x4 matrix, row-major)
  matrix:
    - [0.9999, 0.0087, 0.0012, 45.23]
    - [-0.0087, 0.9999, 0.0045, -12.56]
    - [-0.0012, -0.0045, 1.0000, 85.78]
    - [0.0, 0.0, 0.0, 1.0]

  reprojection_error_mm: 0.35
  num_poses_used: 15

  camera_intrinsics:
    fx: 1200.5
    fy: 1200.8
    cx: 640.0
    cy: 480.0
    distortion: [-0.12, 0.08, 0.0, 0.0, 0.0]
```

---

## 12. Testing

### 12.1 Unit Tests

```cpp
// test_pickplace.cpp
#include <gtest/gtest.h>
#include "PickPlaceManager.h"
#include "MockGripper.h"
#include "MockVision.h"

class PickPlaceTest : public ::testing::Test {
protected:
    void SetUp() override {
        gripper_ = std::make_unique<MockGripperController>();
        vision_ = std::make_unique<MockVisionBridge>();
        trajectory_ = std::make_unique<MockTrajectoryPlanner>();

        manager_ = std::make_unique<PickPlaceManager>(
            *trajectory_, *gripper_, *vision_);
    }

    std::unique_ptr<MockGripperController> gripper_;
    std::unique_ptr<MockVisionBridge> vision_;
    std::unique_ptr<MockTrajectoryPlanner> trajectory_;
    std::unique_ptr<PickPlaceManager> manager_;
};

TEST_F(PickPlaceTest, SingleCycleSuccess) {
    // Setup mocks
    Vision::DetectedObject obj{1, "part", 0.95, 100, 50, 10};
    EXPECT_CALL(*vision_, detectSingleObject(_, _))
        .WillOnce(Return(std::optional(obj)));

    EXPECT_CALL(*gripper_, grip(_)).WillOnce(Return(true));
    EXPECT_CALL(*gripper_, hasPart()).WillOnce(Return(true));
    EXPECT_CALL(*gripper_, release(_)).WillOnce(Return(true));

    EXPECT_CALL(*trajectory_, moveL(_, _))
        .Times(AtLeast(4))
        .WillRepeatedly(Return(true));

    // Run
    bool success = manager_->runSingleCycle();

    // Verify
    EXPECT_TRUE(success);
    EXPECT_EQ(manager_->getStatus().success_count, 1);
}

TEST_F(PickPlaceTest, GripRetryOnFailure) {
    Vision::DetectedObject obj{1, "part", 0.95, 100, 50, 10};
    EXPECT_CALL(*vision_, detectSingleObject(_, _))
        .WillOnce(Return(std::optional(obj)));

    // Grip fails twice, succeeds third time
    EXPECT_CALL(*gripper_, grip(_))
        .WillOnce(Return(false))
        .WillOnce(Return(false))
        .WillOnce(Return(true));

    EXPECT_CALL(*gripper_, hasPart())
        .WillOnce(Return(true));

    PickPlaceConfig config;
    config.max_grip_retries = 3;
    manager_->configure(config);

    bool success = manager_->runSingleCycle();
    EXPECT_TRUE(success);
}

TEST_F(PickPlaceTest, VisionTimeoutError) {
    // Vision returns nothing
    EXPECT_CALL(*vision_, detectSingleObject(_, _))
        .WillRepeatedly(Return(std::nullopt));

    PickPlaceConfig config;
    config.max_scan_retries = 3;
    manager_->configure(config);

    bool success = manager_->runSingleCycle();

    EXPECT_FALSE(success);
    EXPECT_EQ(manager_->getStatus().state, PickPlaceState::IDLE);
    EXPECT_FALSE(manager_->getStatus().last_error.empty());
}
```

### 12.2 Integration Test Checklist

```
Pick & Place Integration Tests:

[ ] Gripper
    [ ] Pneumatic grip/release cycle
    [ ] Part detection sensor
    [ ] Grip timeout handling
    [ ] Vacuum level sensing

[ ] Vision
    [ ] Camera connection
    [ ] Object detection accuracy
    [ ] Pose estimation accuracy
    [ ] Hand-eye calibration validation

[ ] Motion
    [ ] Move to scan position
    [ ] Approach/retract motions
    [ ] Speed limiting
    [ ] Collision avoidance

[ ] Cycle
    [ ] Complete pick-place cycle
    [ ] Retry on grip failure
    [ ] Retry on vision failure
    [ ] Error recovery

[ ] Pallet
    [ ] Pattern generation
    [ ] Position accuracy
    [ ] Layer switching
    [ ] Full pallet cycle

[ ] Safety
    [ ] Vision timeout
    [ ] Gripper force limit
    [ ] Zone restrictions
    [ ] Emergency stop
```

---

## 13. Document History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 0.1 | 2026-02-01 | System | Initial draft |

---

## 14. References

- [ISO 10218-1:2011] Robots and robotic devices — Safety requirements
- [Zivid SDK Documentation] 3D Camera integration
- [OpenCV cv2.calibrateHandEye] Hand-Eye calibration
- CORE_02_Kinematics.md - Forward/Inverse kinematics
- CORE_03_Trajectory.md - Motion planning
- MODE_01_Welding.md - Welding mode (for mode switching)
