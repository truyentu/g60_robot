# PHASE 4: VISION & SENSING TECHNOLOGY
## "Robot Sees" - Hệ thống Thị giác & Cảm biến

**Version:** 1.0
**Ngày tạo:** 01/02/2026
**Trạng thái:** DRAFT
**Phụ thuộc:** Phase 3 (Welding Technology) phải hoàn thành

---

## 1. TỔNG QUAN & PHẠM VI

### 1.1 Mục tiêu Phase 4
Xây dựng hệ thống thị giác và cảm biến hoàn chỉnh cho robot hàn, cho phép:
- **Seam Tracking (Real-time):** Bám đường hàn thời gian thực trong quá trình hàn
- **Seam Finding (Offline):** Quét và phát hiện đường hàn trước khi hàn (Scan-to-Path)
- **Touch Sensing:** Dò tìm điểm bắt đầu hàn bằng tiếp xúc điện
- **TCP Calibration:** Hiệu chuẩn Tool Center Point
- **Hand-Eye Calibration:** Hiệu chuẩn tay-mắt cho cảm biến gắn trên tay robot

### 1.2 Milestone: "Robot Sees"
**Tiêu chí hoàn thành:**
- Robot quét phôi và tự động phát hiện đường hàn V-Groove/Fillet
- Seam Tracking bù sai lệch ±5mm với độ trễ < 50ms
- Touch Sensing phát hiện điểm tiếp xúc với độ chính xác ±0.5mm
- TCP Calibration với 4-point method đạt sai số < 0.1mm

### 1.3 Kiến trúc Hệ thống Vision

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        VISION SYSTEM ARCHITECTURE                        │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │                    SENSOR LAYER (Hardware)                        │   │
│  │  ┌────────────────┐  ┌────────────────┐  ┌──────────────────┐   │   │
│  │  │  Blue Laser    │  │  Touch Sense   │  │   Encoder        │   │   │
│  │  │  Profiler 2D   │  │  Circuit       │  │   (Trigger)      │   │   │
│  │  │  (405-450nm)   │  │  (24V DC)      │  │                  │   │   │
│  │  └───────┬────────┘  └───────┬────────┘  └────────┬─────────┘   │   │
│  └──────────┼───────────────────┼───────────────────┼──────────────┘   │
│             │ GigE Vision       │ Digital I/O       │ Hardware Trigger │
│             ▼                   ▼                   ▼                  │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │                  ACQUISITION LAYER (C++ Native)                   │   │
│  │  ┌────────────────┐  ┌────────────────┐  ┌──────────────────┐   │   │
│  │  │ SensorDriver   │  │ TouchSense     │  │   SyncManager    │   │   │
│  │  │ (GigE/GenICam) │  │ Monitor        │  │   (Encoder Sync) │   │   │
│  │  └───────┬────────┘  └───────┬────────┘  └────────┬─────────┘   │   │
│  └──────────┼───────────────────┼───────────────────┼──────────────┘   │
│             │                   │                   │                  │
│             ▼                   ▼                   ▼                  │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │                  PROCESSING LAYER (C++ / PCL)                     │   │
│  │  ┌────────────────┐  ┌────────────────┐  ┌──────────────────┐   │   │
│  │  │ NoiseFilter    │  │ FeatureExtract │  │   PathPlanner    │   │   │
│  │  │ (SOR/Intensity)│  │ (RANSAC/Steger)│  │   (B-Spline)     │   │   │
│  │  └───────┬────────┘  └───────┬────────┘  └────────┬─────────┘   │   │
│  └──────────┼───────────────────┼───────────────────┼──────────────┘   │
│             │                   │                   │                  │
│             ▼                   ▼                   ▼                  │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │                  CONTROL LAYER (C# + Ruckig)                      │   │
│  │  ┌────────────────┐  ┌────────────────┐  ┌──────────────────┐   │   │
│  │  │ SeamTracker    │  │ CalibManager   │  │   VisionHMI      │   │   │
│  │  │ (Real-time)    │  │ (Hand-Eye/TCP) │  │   (WPF/Helix)    │   │   │
│  │  └────────────────┘  └────────────────┘  └──────────────────┘   │   │
│  └──────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────┘
```

### 1.4 Nguyên tắc Thiết kế

| Nguyên tắc | Mô tả |
|------------|-------|
| **Blue Laser** | Sử dụng laser xanh 405-450nm để giảm nhiễu từ bề mặt kim loại nóng |
| **Hardware Trigger** | Đồng bộ encoder thay vì nội suy thời gian để đảm bảo độ chính xác |
| **Hybrid C++/C#** | Processing nặng bằng C++ (PCL), UI và logic bằng C# |
| **Zero-copy** | Truyền dữ liệu thông qua pointer để giảm latency |

---

## 2. CẤU TRÚC THƯ MỤC

### 2.1 Thư mục mới cần tạo

```
src/
├── core/
│   ├── vision/                         # [NEW] Module Vision chính
│   │   ├── sensor/                     # Driver cảm biến
│   │   │   ├── ISensorDriver.hpp       # Interface chung
│   │   │   ├── LaserProfilerDriver.hpp # Driver cho Laser Profiler
│   │   │   ├── LaserProfilerDriver.cpp
│   │   │   ├── GigEVisionClient.hpp    # GigE Vision protocol
│   │   │   └── HikrobotMVS.hpp         # Hikrobot MVS SDK wrapper
│   │   │
│   │   ├── processing/                 # Xử lý dữ liệu
│   │   │   ├── PointCloudProcessor.hpp # Xử lý point cloud (PCL wrapper)
│   │   │   ├── NoiseFilter.hpp         # Bộ lọc nhiễu (SOR, ROR, Intensity)
│   │   │   ├── FeatureExtractor.hpp    # Trích xuất đặc trưng
│   │   │   ├── VGrooveDetector.hpp     # Phát hiện rãnh V
│   │   │   ├── FilletDetector.hpp      # Phát hiện mối hàn góc
│   │   │   └── LaserCenterExtract.hpp  # Steger algorithm
│   │   │
│   │   ├── tracking/                   # Seam Tracking
│   │   │   ├── SeamTracker.hpp         # Real-time seam tracking
│   │   │   ├── SeamTracker.cpp
│   │   │   ├── TrackingFilter.hpp      # Kalman filter cho tracking
│   │   │   └── LatencyCompensator.hpp  # Bù trễ hệ thống
│   │   │
│   │   ├── calibration/                # Hiệu chuẩn
│   │   │   ├── HandEyeCalibration.hpp  # Hand-Eye calibration (AX=XB)
│   │   │   ├── TCPCalibration.hpp      # TCP 4-point method
│   │   │   └── SensorCalibration.hpp   # Laser sensor calibration
│   │   │
│   │   └── sync/                       # Đồng bộ dữ liệu
│   │       ├── SyncManager.hpp         # Đồng bộ sensor-robot
│   │       ├── EncoderTrigger.hpp      # Hardware trigger
│   │       └── TimestampBuffer.hpp     # Buffer đồng bộ thời gian
│   │
│   ├── touch/                          # [NEW] Touch Sensing
│   │   ├── TouchSenseController.hpp    # Điều khiển touch sensing
│   │   ├── TouchSenseController.cpp
│   │   ├── TouchPattern.hpp            # Các pattern dò tìm
│   │   └── TouchCalibration.hpp        # Hiệu chuẩn touch offset
│   │
│   └── coordinate/                     # [NEW] Biến đổi tọa độ
│       ├── TransformChain.hpp          # Chuỗi biến đổi tọa độ
│       ├── FrameManager.hpp            # Quản lý các frame
│       ├── EulerConversion.hpp         # Chuyển đổi Euler angles
│       └── QuaternionUtils.hpp         # Quaternion utilities
│
├── native/                             # [NEW] C++ Native DLL
│   ├── VisionCore/                     # PCL-based processing
│   │   ├── VisionCore.hpp              # Export interface
│   │   ├── VisionCore.cpp
│   │   ├── PCLWrapper.hpp              # PCL wrapper classes
│   │   └── CMakeLists.txt              # CMake build
│   │
│   └── SensorSDK/                      # Sensor SDK wrappers
│       ├── HikrobotWrapper.hpp
│       └── RiftekWrapper.hpp
│
├── ui/
│   └── vision/                         # [NEW] Vision UI
│       ├── VisionPanel.xaml            # Panel chính
│       ├── VisionPanel.xaml.cs
│       ├── PointCloudView.xaml         # Hiển thị point cloud (Helix)
│       ├── LaserProfileView.xaml       # Hiển thị profile 2D
│       ├── CalibrationWizard.xaml      # Wizard hiệu chuẩn
│       └── TrackingMonitor.xaml        # Monitor seam tracking
│
└── tests/
    └── vision/                         # [NEW] Tests
        ├── test_FeatureExtractor.cpp
        ├── test_SeamTracker.cpp
        └── test_Calibration.cpp
```

---

## 3. DANH SÁCH TASK CHI TIẾT

### 3.1 Nhóm A: Sensor Interface (P4-01 → P4-04)

#### P4-01: GigE Vision Client
**Mô tả:** Xây dựng client GigE Vision để giao tiếp với Laser Profiler.

**Giao thức GigE Vision:**
```
┌──────────────┐         ┌──────────────┐
│     PC       │◀───────▶│   Sensor     │
│  (Client)    │  GigE   │  (Server)    │
└──────────────┘         └──────────────┘
      │                        │
      │  1. GVCP Discovery     │
      │ ──────────────────────▶│
      │                        │
      │  2. GVCP Ack           │
      │ ◀──────────────────────│
      │                        │
      │  3. GVSP Stream Start  │
      │ ──────────────────────▶│
      │                        │
      │  4. Profile Data (UDP) │
      │ ◀──────────────────────│
      └────────────────────────┘
```

**Interface:**
```cpp
class IGigEVisionClient {
public:
    virtual bool discover(std::vector<SensorInfo>& sensors) = 0;
    virtual bool connect(const std::string& ip) = 0;
    virtual void disconnect() = 0;

    virtual bool startStreaming() = 0;
    virtual bool stopStreaming() = 0;

    virtual void setCallback(ProfileCallback cb) = 0;

    // GenICam feature access
    virtual bool setFeature(const std::string& name, int value) = 0;
    virtual bool getFeature(const std::string& name, int& value) = 0;
};
```

**DoD:**
- [ ] Discover sensors trên network
- [ ] Nhận profile data qua UDP streaming
- [ ] Hỗ trợ Jumbo Frames (MTU 9000)
- [ ] Unit test với mock sensor

---

#### P4-02: Laser Profiler Driver (Hikrobot MVS)
**Mô tả:** Wrapper cho Hikrobot MVS SDK để điều khiển Laser Profiler.

**SDK Integration:**
```cpp
class HikrobotLaserDriver : public ILaserProfilerDriver {
public:
    // Initialization
    bool initialize();
    bool openDevice(const std::string& serialNumber);

    // Streaming
    bool startAcquisition();
    bool stopAcquisition();

    // Trigger mode
    void setTriggerMode(TriggerMode mode);  // FREE_RUN, SOFTWARE, HARDWARE
    void setEncoderDivider(int divider);    // Xung trigger / profile

    // Callback
    void registerCallback(ProfileCallback cb);

    // Profile data
    struct ProfileData {
        std::vector<float> x;       // X coordinates (mm)
        std::vector<float> z;       // Z coordinates (depth, mm)
        std::vector<uint16_t> intensity;  // Intensity values
        uint64_t timestamp;
        uint32_t encoderCount;
    };

private:
    void* m_handle = nullptr;
    ProfileCallback m_callback;
};
```

**Chế độ Trigger:**
| Mode | Mô tả | Use Case |
|------|-------|----------|
| FREE_RUN | Chụp liên tục theo tần số cố định | Testing, Calibration |
| SOFTWARE | Trigger bằng lệnh software | Scan-to-Path |
| HARDWARE | Trigger bằng encoder | Seam Tracking |

**DoD:**
- [ ] Kết nối và nhận data từ sensor thực
- [ ] Hỗ trợ cả 3 chế độ trigger
- [ ] Cấu hình encoder divider
- [ ] Documentation cho từng sensor model

---

#### P4-03: Encoder Synchronization
**Mô tả:** Đồng bộ dữ liệu sensor với vị trí robot bằng hardware trigger.

**Vấn đề với Time-based Interpolation:**
```
Time:     t0        t1        t2        t3
          │         │         │         │
Robot:    P0────────P1────────P2────────P3
                    ╲
                     ╲ Jitter ±10-20ms
                      ╲
Sensor:   S0─────S1─────S2─────S3
          │     │       │      │
          ▼     ▼       ▼      ▼
Error:    OK    ±0.2mm  ±0.4mm ±0.6mm  (at 20mm/s)
```

**Giải pháp Hardware Trigger:**
```
Robot Motion ──▶ Encoder Pulses ──▶ Camera Trigger
                                          │
                                          ▼
              ┌─────────────────────────────────────┐
              │  Profile captured at EXACT position │
              │  (independent of velocity)          │
              └─────────────────────────────────────┘
```

**Interface:**
```cpp
class EncoderSyncManager {
public:
    // Configuration
    void setEncoderResolution(double pulses_per_mm);
    void setProfileSpacing(double mm);  // Desired spacing between profiles

    // Synchronization
    void startSync();
    void stopSync();

    // Get robot position at encoder count
    Eigen::Affine3d getRobotPoseAtCount(uint32_t encoderCount);

    // Callback when profile is ready with synchronized pose
    void onSyncedProfile(std::function<void(const ProfileData&, const Eigen::Affine3d&)> cb);

private:
    RingBuffer<std::pair<uint32_t, Eigen::Affine3d>> m_poseBuffer;
    double m_pulsesPerMm = 1000.0;
    double m_profileSpacing = 0.5;  // mm
};
```

**DoD:**
- [ ] Trigger camera từ encoder signal
- [ ] Buffer robot poses với encoder count
- [ ] Lookup pose chính xác cho mỗi profile
- [ ] Test với các tốc độ robot khác nhau

---

#### P4-04: Sensor Data Buffer
**Mô tả:** Ring buffer an toàn thread cho dữ liệu sensor.

**Architecture:**
```
┌─────────────┐     ┌─────────────────────────────┐     ┌─────────────┐
│   Sensor    │────▶│      Ring Buffer            │────▶│  Processing │
│   Thread    │     │  [0][1][2][3][4][5][6][7]   │     │   Thread    │
└─────────────┘     └─────────────────────────────┘     └─────────────┘
     Producer              Lock-free                        Consumer
```

**Interface:**
```cpp
template<typename T, size_t Size>
class LockFreeRingBuffer {
public:
    bool push(const T& item);
    bool pop(T& item);
    bool isEmpty() const;
    bool isFull() const;
    size_t size() const;

private:
    std::array<T, Size> m_buffer;
    std::atomic<size_t> m_head{0};
    std::atomic<size_t> m_tail{0};
};
```

**DoD:**
- [ ] Lock-free implementation
- [ ] No memory allocation during operation
- [ ] Overflow handling (drop oldest or reject new)
- [ ] Performance test: >100k profiles/sec

---

### 3.2 Nhóm B: Point Cloud Processing (P4-05 → P4-09)

#### P4-05: Noise Filter (SOR + Intensity)
**Mô tả:** Lọc nhiễu từ dữ liệu point cloud sử dụng PCL.

**Loại nhiễu trong môi trường hàn:**
| Loại nhiễu | Nguyên nhân | Giải pháp |
|------------|-------------|-----------|
| Speckle | Giao thoa laser trên bề mặt nhám | SOR filter |
| Outliers | Bụi, khói hàn | Radius Outlier Removal |
| Reflection | Phản xạ gương trên kim loại bóng | Intensity filter |
| Multipath | Phản xạ đa đường | Intensity filter |

**Statistical Outlier Removal (SOR):**
```cpp
// Loại bỏ điểm có khoảng cách trung bình đến k neighbors > mean + α*stddev
void filterSOR(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
               int k_neighbors = 50,
               double stddev_mult = 1.0) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(k_neighbors);
    sor.setStddevMulThresh(stddev_mult);
    sor.filter(*cloud);
}
```

**Intensity Filter:**
```cpp
// Loại bỏ điểm có intensity bão hòa hoặc quá thấp
void filterIntensity(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                     uint16_t min_intensity = 10,
                     uint16_t max_intensity = 250) {
    pcl::ConditionAnd<pcl::PointXYZI>::Ptr condition(new pcl::ConditionAnd<pcl::PointXYZI>());
    condition->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZI>("intensity", pcl::ComparisonOps::GT, min_intensity)));
    condition->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZI>("intensity", pcl::ComparisonOps::LT, max_intensity)));

    pcl::ConditionalRemoval<pcl::PointXYZI> filter;
    filter.setCondition(condition);
    filter.setInputCloud(cloud);
    filter.filter(*cloud);
}
```

**DoD:**
- [ ] SOR filter với configurable parameters
- [ ] Intensity-based filter
- [ ] Combined filter pipeline
- [ ] Benchmark: <5ms cho 10k points

---

#### P4-06: Coordinate Transformation Chain
**Mô tả:** Xây dựng chuỗi biến đổi tọa độ từ sensor → robot base.

**Transformation Chain:**
```
P_base = T_base_flange * T_flange_sensor * P_sensor

Trong đó:
- P_sensor:        Điểm trong hệ tọa độ cảm biến
- T_flange_sensor: Ma trận Hand-Eye (cố định, từ calibration)
- T_base_flange:   Ma trận động học thuận (thay đổi theo vị trí robot)
- P_base:          Điểm trong hệ tọa độ gốc robot
```

**Implementation:**
```cpp
class TransformChain {
public:
    // Set transforms
    void setHandEyeTransform(const Eigen::Affine3d& T_flange_sensor);
    void setToolTransform(const Eigen::Affine3d& T_flange_tool);

    // Update robot pose (from controller)
    void updateRobotPose(const Eigen::Affine3d& T_base_flange);

    // Transform points
    Eigen::Vector3d sensorToBase(const Eigen::Vector3d& p_sensor) const;
    Eigen::Vector3d baseToSensor(const Eigen::Vector3d& p_base) const;
    Eigen::Vector3d sensorToTool(const Eigen::Vector3d& p_sensor) const;

    // Transform point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_sensor) const;

private:
    Eigen::Affine3d m_T_flange_sensor;  // Hand-Eye
    Eigen::Affine3d m_T_flange_tool;    // Tool offset
    Eigen::Affine3d m_T_base_flange;    // Current robot pose
};
```

**DoD:**
- [ ] Chuyển đổi đúng giữa các frame
- [ ] Thread-safe cho concurrent access
- [ ] Validation với known transforms
- [ ] Unit test với synthetic data

---

#### P4-07: V-Groove Detector (RANSAC)
**Mô tả:** Phát hiện đường hàn rãnh V sử dụng RANSAC trên profile 2D.

**Thuật toán:**
```
Profile 2D (Z vs X)
    │
    ▼
┌────────────────────────┐
│  1. Split profile      │ ──▶ Left half + Right half
└────────────────────────┘
    │
    ▼
┌────────────────────────┐
│  2. RANSAC fit line    │ ──▶ Line_left, Line_right
│     for each half      │
└────────────────────────┘
    │
    ▼
┌────────────────────────┐
│  3. Find intersection  │ ──▶ Root point (X_root, Z_root)
└────────────────────────┘
    │
    ▼
┌────────────────────────┐
│  4. Calculate groove   │ ──▶ Angle, Width, Depth
│     parameters         │
└────────────────────────┘
```

**Interface:**
```cpp
struct VGrooveFeature {
    Eigen::Vector2d rootPoint;      // Điểm đáy rãnh
    Eigen::Vector2d leftShoulder;   // Vai trái
    Eigen::Vector2d rightShoulder;  // Vai phải
    double grooveAngle;             // Góc rãnh (degrees)
    double grooveWidth;             // Chiều rộng rãnh (mm)
    double grooveDepth;             // Chiều sâu rãnh (mm)
    double confidence;              // Độ tin cậy (0-1)
};

class VGrooveDetector {
public:
    void setRANSACParams(double distanceThreshold, int maxIterations);
    void setGrooveConstraints(double minAngle, double maxAngle);

    bool detect(const std::vector<Eigen::Vector2d>& profile,
                VGrooveFeature& feature);

    // Multi-profile detection (3D)
    bool detectPath(const std::vector<std::vector<Eigen::Vector2d>>& profiles,
                    std::vector<Eigen::Vector3d>& seamPath);
};
```

**DoD:**
- [ ] Phát hiện V-Groove với góc 30-90 độ
- [ ] Loại bỏ outliers tự động
- [ ] Confidence score dựa trên inlier ratio
- [ ] Processing < 10ms per profile

---

#### P4-08: Fillet Detector (Plane Intersection)
**Mô tả:** Phát hiện mối hàn góc bằng cách tìm giao tuyến hai mặt phẳng.

**Thuật toán RANSAC Plane Fitting:**
```cpp
bool FilletDetector::detect(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    FilletFeature& feature)
{
    // 1. Fit first plane (dominant plane)
    pcl::ModelCoefficients::Ptr plane1(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers1(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(m_distanceThreshold);
    seg.setInputCloud(cloud);
    seg.segment(*inliers1, *plane1);

    // 2. Remove inliers, fit second plane
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers1);
    extract.setNegative(true);

    pcl::PointCloud<pcl::PointXYZ>::Ptr remaining(new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter(*remaining);

    pcl::ModelCoefficients::Ptr plane2(new pcl::ModelCoefficients);
    seg.setInputCloud(remaining);
    seg.segment(*inliers2, *plane2);

    // 3. Calculate intersection line
    Eigen::Vector3d n1(plane1->values[0], plane1->values[1], plane1->values[2]);
    Eigen::Vector3d n2(plane2->values[0], plane2->values[1], plane2->values[2]);

    feature.direction = n1.cross(n2).normalized();  // Seam direction
    feature.point = findIntersectionPoint(plane1, plane2);
    feature.angle = std::acos(n1.dot(n2)) * 180.0 / M_PI;

    return true;
}
```

**DoD:**
- [ ] Phát hiện Fillet với góc 45-135 độ
- [ ] Trả về direction và point của seam
- [ ] Xử lý được trường hợp nhiều mặt phẳng
- [ ] Validation với CAD model

---

#### P4-09: Steger Line Center Extraction
**Mô tả:** Trích xuất tâm đường laser với độ chính xác sub-pixel.

**Thuật toán Steger:**
```
Input: Grayscale image của đường laser

1. Gaussian Smoothing
   Làm mịn ảnh để giảm nhiễu

2. Compute Hessian Matrix
   H = [Ixx  Ixy]
       [Ixy  Iyy]

3. Find Eigenvalues & Eigenvectors
   Tại mỗi pixel, tìm hướng vuông góc với đường laser

4. Sub-pixel Position
   Sử dụng Taylor expansion để nội suy vị trí cực trị
   t = -Ip / Ipp  (đạo hàm bậc 1 / đạo hàm bậc 2)

5. Extract Center Line
   Nối các điểm cực trị thành đường
```

**Interface:**
```cpp
class StegerLineExtractor {
public:
    void setGaussianSigma(double sigma);
    void setThresholds(double minResponse, double maxWidth);

    // Extract center line from image
    bool extract(const cv::Mat& laserImage,
                 std::vector<Eigen::Vector2d>& centerLine);

    // Sub-pixel accuracy
    double getSubPixelAccuracy() const { return m_subPixelAccuracy; }

private:
    double computeHessianResponse(const cv::Mat& Ixx, const cv::Mat& Iyy,
                                   const cv::Mat& Ixy, int x, int y);
    Eigen::Vector2d refinePosition(const cv::Mat& image, int x, int y);

    double m_sigma = 2.0;
    double m_minResponse = 0.5;
    double m_subPixelAccuracy = 0.1;  // pixels
};
```

**DoD:**
- [ ] Sub-pixel accuracy < 0.1 pixel
- [ ] Xử lý được đường laser bị đứt
- [ ] Loại bỏ reflection artifacts
- [ ] Benchmark: 640x480 image < 5ms

---

### 3.3 Nhóm C: Seam Tracking (P4-10 → P4-13)

#### P4-10: Real-time Seam Tracker
**Mô tả:** Module bám đường hàn thời gian thực trong quá trình hàn.

**Control Loop (100Hz):**
```
┌─────────────────────────────────────────────────────────────┐
│                    SEAM TRACKING LOOP                        │
│                                                              │
│  1. Acquire Profile ───▶ 2. Extract Features ───┐           │
│         │                                        │           │
│         │                                        ▼           │
│  6. Send to Robot ◀─── 5. Update Ruckig ◀─── 3. Calculate  │
│         │                                    Error Vector    │
│         │                                        │           │
│         ▼                                        │           │
│  7. Robot Executes                              │           │
│         │                                        │           │
│         └───────────────────────────────────────┘           │
│                     4. Apply Filter (Kalman)                 │
│                                                              │
│  Target: < 20ms total latency                               │
└─────────────────────────────────────────────────────────────┘
```

**Interface:**
```cpp
class SeamTracker {
public:
    // Configuration
    void setTrackingMode(TrackingMode mode);  // AHEAD, COAXIAL
    void setLookAheadDistance(double mm);     // Khoảng cách đầu đo trước mỏ hàn
    void setMaxCorrection(double mm);         // Giới hạn bù tối đa

    // Enable/Disable
    void enable();
    void disable();
    bool isEnabled() const;

    // Update (called from sensor callback)
    void onNewProfile(const ProfileData& profile, const Eigen::Affine3d& robotPose);

    // Get correction
    Eigen::Vector3d getCorrectionVector() const;

    // Callbacks
    void onCorrectionReady(std::function<void(const Eigen::Vector3d&)> cb);
    void onTrackingLost(std::function<void()> cb);

    // Diagnostics
    struct TrackingStatus {
        bool isTracking;
        double errorX, errorY, errorZ;
        double confidence;
        uint32_t lostFrameCount;
    };
    TrackingStatus getStatus() const;

private:
    FeatureExtractor m_extractor;
    KalmanFilter m_filter;
    double m_lookAheadDistance = 50.0;  // mm
    double m_maxCorrection = 5.0;       // mm
};
```

**DoD:**
- [ ] Track V-Groove và Fillet seams
- [ ] Correction vector update ở 100Hz
- [ ] Kalman filter cho smooth tracking
- [ ] Tracking loss detection

---

#### P4-11: Kalman Filter for Tracking
**Mô tả:** Bộ lọc Kalman để làm mịn dữ liệu tracking.

**State Vector:**
```
State x = [px, py, pz, vx, vy, vz]^T
         (position)   (velocity)

Measurement z = [px, py, pz]^T
               (từ feature extraction)
```

**Implementation:**
```cpp
class TrackingKalmanFilter {
public:
    void initialize(const Eigen::Vector3d& initialPosition);

    // Process model: constant velocity
    void setProcessNoise(double posNoise, double velNoise);

    // Measurement model
    void setMeasurementNoise(double noise);

    // Update cycle
    void predict(double dt);
    void update(const Eigen::Vector3d& measurement);

    // Get state
    Eigen::Vector3d getPosition() const;
    Eigen::Vector3d getVelocity() const;
    Eigen::Matrix3d getPositionCovariance() const;

private:
    Eigen::VectorXd m_x;  // State [6x1]
    Eigen::MatrixXd m_P;  // Covariance [6x6]
    Eigen::MatrixXd m_Q;  // Process noise
    Eigen::MatrixXd m_R;  // Measurement noise
};
```

**DoD:**
- [ ] Làm mịn measurement noise
- [ ] Dự đoán position khi mất data
- [ ] Tunable noise parameters
- [ ] Unit test với synthetic trajectory

---

#### P4-12: Latency Compensation
**Mô tả:** Bù trễ hệ thống để correction đúng thời điểm.

**Latency Sources:**
```
t_total = t_acquisition + t_processing + t_communication + t_execution

Typical values:
- t_acquisition:   10ms (camera exposure + transfer)
- t_processing:    5ms  (feature extraction)
- t_communication: 5ms  (PC → Teensy)
- t_execution:     10ms (motion execution)
────────────────────────────────────────────────────
Total:             30ms
```

**Compensation Strategy:**
```cpp
class LatencyCompensator {
public:
    void setTotalLatency(double ms);

    // Compensate correction based on robot velocity
    Eigen::Vector3d compensate(const Eigen::Vector3d& correction,
                               const Eigen::Vector3d& velocity) {
        // Shift correction by latency * velocity
        double dt = m_totalLatency / 1000.0;  // Convert to seconds
        return correction + velocity * dt;
    }

    // Auto-measure latency
    void measureLatency(RobotController& robot, LaserProfiler& sensor);

private:
    double m_totalLatency = 30.0;  // ms
};
```

**DoD:**
- [ ] Tính toán tổng latency chính xác
- [ ] Bù correction dựa trên velocity
- [ ] Auto-measure latency function
- [ ] Validation với oscilloscope

---

#### P4-13: Ruckig Integration for Tracking
**Mô tả:** Tích hợp Ruckig OTG để tạo quỹ đạo smooth từ correction.

**Strategy: Target State Update**
```cpp
// Thay vì cộng trực tiếp correction vào position,
// cập nhật target state của Ruckig và để nó tính quỹ đạo smooth

void SeamTracker::applyCorrection(const Eigen::Vector3d& correction) {
    // Get current Ruckig state
    auto currentTarget = m_ruckig->getCurrentTarget();

    // Update target with correction
    Eigen::Vector3d newTarget = currentTarget.position + correction;

    // Set new target (Ruckig will plan smooth transition)
    m_ruckig->setTarget(newTarget, m_maxVelocity, m_maxAcceleration);

    // Ruckig automatically handles jerk-limited motion
}
```

**DoD:**
- [ ] Smooth correction không gây jerk
- [ ] Respect velocity/acceleration limits
- [ ] Handle rapid correction changes
- [ ] Integration test với motion system

---

### 3.4 Nhóm D: Calibration (P4-14 → P4-17)

#### P4-14: Hand-Eye Calibration (AX=XB)
**Mô tả:** Hiệu chuẩn tay-mắt để tìm ma trận T_flange_sensor.

**Phương pháp:**
```
Eye-in-Hand Configuration:
Robot moves → Camera moves with it → Views fixed target

Equation: A*X = X*B
Where:
- A = T_flange1^-1 * T_flange2  (Robot motion)
- B = T_target1 * T_target2^-1  (Camera motion)
- X = T_flange_sensor           (Unknown, to be found)
```

**Quy trình:**
```
1. Đặt target cố định trong không gian

2. Di chuyển robot đến N vị trí khác nhau (N >= 3)
   - Tại mỗi vị trí, ghi lại:
     - T_base_flange (từ robot controller)
     - T_sensor_target (từ camera/sensor)

3. Tính toán các cặp (A_i, B_i) từ chuyển động tương đối

4. Giải hệ phương trình AX = XB
   - Phương pháp: Tsai-Lenz, Park-Martin, hoặc Daniilidis
```

**Interface:**
```cpp
class HandEyeCalibration {
public:
    enum Method {
        TSAI_LENZ,      // Classic method
        PARK_MARTIN,    // Dual quaternion
        DANIILIDIS,     // SVD-based
        ANDREFF         // Linear method
    };

    void setMethod(Method method);

    // Add calibration pose pair
    void addPose(const Eigen::Affine3d& T_base_flange,
                 const Eigen::Affine3d& T_sensor_target);

    // Calibrate
    bool calibrate(Eigen::Affine3d& T_flange_sensor);

    // Get error metrics
    double getRotationError() const;    // degrees
    double getTranslationError() const; // mm

    // Validation
    bool validate(const std::vector<Eigen::Affine3d>& testPoses);

private:
    std::vector<std::pair<Eigen::Affine3d, Eigen::Affine3d>> m_poses;
    Method m_method = PARK_MARTIN;
};
```

**DoD:**
- [ ] Implement ít nhất 2 methods
- [ ] Rotation error < 0.5 degrees
- [ ] Translation error < 1mm
- [ ] Validation routine included

---

#### P4-15: TCP Calibration (4-Point Method)
**Mô tả:** Hiệu chuẩn Tool Center Point bằng phương pháp 4 điểm.

**Nguyên lý:**
```
Tất cả các vị trí robot khác nhau đều chỉ về cùng một điểm TCP cố định trong không gian:

P_tcp = T_base_flange * T_flange_tcp

Với N vị trí robot (N >= 4), giải hệ phương trình:
T_flange1 * P_tcp = T_flange2 * P_tcp = ... = T_flangeN * P_tcp
```

**Interface:**
```cpp
class TCPCalibration {
public:
    enum Method {
        FOUR_POINT,     // Standard 4-point
        ROTATION_AXIS,  // Using rotation axis
        SPHERE_FIT      // Fit sphere to tool tip
    };

    void setMethod(Method method);

    // Add calibration pose
    void addPose(const Eigen::Affine3d& T_base_flange);

    // Calibrate
    bool calibrate(Eigen::Vector3d& P_tcp);

    // Get error
    double getError() const;  // mm (radius of best-fit sphere)

    // Interactive mode
    void startInteractiveCalibration(RobotController& robot);

private:
    std::vector<Eigen::Affine3d> m_poses;
    Method m_method = FOUR_POINT;
};
```

**DoD:**
- [ ] 4-point method với error < 0.5mm
- [ ] Auto-detect khi có đủ poses
- [ ] Sphere fit visualization
- [ ] Integration với robot controller

---

#### P4-16: Sensor Calibration Wizard
**Mô tả:** Wizard UI hướng dẫn từng bước calibration.

**Wizard Flow:**
```
┌─────────────────────────────────────────────────────────────┐
│  CALIBRATION WIZARD                                          │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  Step 1: TCP Calibration                                     │
│  ├─ Move robot to position 1 → [Capture]                    │
│  ├─ Move robot to position 2 → [Capture]                    │
│  ├─ Move robot to position 3 → [Capture]                    │
│  └─ Move robot to position 4 → [Capture] → [Calculate]      │
│                                                              │
│  Step 2: Hand-Eye Calibration                                │
│  ├─ Place target in view                                     │
│  ├─ Move robot to position 1 → [Capture]                    │
│  ├─ ...                                                      │
│  └─ Move robot to position N → [Capture] → [Calculate]      │
│                                                              │
│  Step 3: Validation                                          │
│  ├─ Move to test position → [Measure Error]                 │
│  └─ [Accept] / [Retry]                                       │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

**DoD:**
- [ ] Step-by-step UI wizard
- [ ] Real-time preview trong mỗi step
- [ ] Error display và validation
- [ ] Save/Load calibration data

---

#### P4-17: Calibration Data Management
**Mô tả:** Lưu trữ và quản lý dữ liệu calibration.

**File Format (YAML):**
```yaml
calibration:
  date: "2026-02-01T10:30:00"
  robot_model: "Puma560"
  sensor_model: "Hikrobot MV-DP2025"

  tcp:
    method: "FOUR_POINT"
    result:
      x: 150.23
      y: 0.15
      z: 85.67
    error_mm: 0.12
    poses_used: 4

  hand_eye:
    method: "PARK_MARTIN"
    result:
      translation: [50.1, 20.3, -30.5]
      rotation_euler_xyz: [0.5, -0.3, 179.8]
    rotation_error_deg: 0.25
    translation_error_mm: 0.45
    poses_used: 8

  sensor:
    working_distance_mm: 200.0
    field_of_view_mm: 100.0
    resolution_mm: 0.05
```

**DoD:**
- [ ] Save/Load calibration data
- [ ] Versioning và timestamp
- [ ] Validate data integrity
- [ ] Export to different formats

---

### 3.5 Nhóm E: Touch Sensing (P4-18 → P4-20)

#### P4-18: Touch Sense Circuit Interface
**Mô tả:** Giao tiếp với mạch touch sensing điện.

**Nguyên lý:**
```
┌─────────────────────────────────────────────────────────────┐
│                    TOUCH SENSING CIRCUIT                     │
│                                                              │
│    Robot                                                     │
│    Controller    ┌──────────┐     ┌──────────┐              │
│        │         │  Current │     │  Metal   │              │
│        │         │  Source  │     │ Workpiece│              │
│        ▼         │  (24V)   │     │          │              │
│    ┌──────┐      └────┬─────┘     └────┬─────┘              │
│    │Teensy│◀─────────┐│                │                    │
│    │ DI   │          ││    Wire Tip    │                    │
│    └──────┘          │└───────●────────┘                    │
│                      │        │                              │
│                      │   ┌────┴────┐                         │
│                      │   │ Contact │  ◀── Touch Event       │
│                      │   │ Detect  │                         │
│                      │   └─────────┘                         │
│                      │        │                              │
│                      └────────┴──▶ DI_TOUCH_SIGNAL           │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

**Interface:**
```cpp
class TouchSenseController {
public:
    // Initialization
    void initialize(DigitalIO& dio);
    void setTouchPin(int pin);

    // Configuration
    void setDebounceTime(uint32_t ms);
    void setRetractDistance(double mm);  // Khoảng rút lại sau touch

    // Touch detection
    bool isTouched() const;

    // Callback
    void onTouch(std::function<void()> cb);
    void onTouchLost(std::function<void()> cb);

private:
    DigitalIO* m_dio = nullptr;
    int m_touchPin = 0;
    uint32_t m_debounceTime = 10;
    bool m_lastState = false;
};
```

**DoD:**
- [ ] Debounced touch detection
- [ ] Callback on touch events
- [ ] Configurable retract distance
- [ ] Integration với robot motion

---

#### P4-19: Touch Search Patterns
**Mô tả:** Các pattern di chuyển để dò tìm điểm tiếp xúc.

**Các Pattern:**
```
1. LINEAR SEARCH (1D)
   ────────────────▶ Direction
   Start ●────────────────●──X Touch

2. CROSS PATTERN (2D)
           │
           │
   ────────●────────
           │
           │

3. SPIRAL SEARCH (2D)
          ╭───────╮
   Start ●│       │
          ╰───────┼───╮
                  │   │
          ╭───────┼───╯
          │       │
          ╰───────╯

4. ZIGZAG PATTERN (Surface Scan)
   ●───────────────────────▶
   ◀───────────────────────●
   ●───────────────────────▶
   ◀───────────────────────●
```

**Interface:**
```cpp
enum class TouchPattern {
    LINEAR,
    CROSS,
    SPIRAL,
    ZIGZAG
};

class TouchSearcher {
public:
    void setPattern(TouchPattern pattern);
    void setSearchArea(double width, double height);
    void setStepSize(double mm);
    void setApproachSpeed(double mm_per_s);

    // Execute search
    bool startSearch(const Eigen::Vector3d& startPos,
                     const Eigen::Vector3d& direction);

    // Result
    bool getTouchPoint(Eigen::Vector3d& point) const;

    // Callbacks
    void onProgress(std::function<void(double percent)> cb);
    void onComplete(std::function<void(bool success, const Eigen::Vector3d& point)> cb);

private:
    TouchPattern m_pattern = TouchPattern::LINEAR;
    TouchSenseController m_touchController;
    double m_stepSize = 1.0;     // mm
    double m_approachSpeed = 5.0; // mm/s
};
```

**DoD:**
- [ ] Implement 4 search patterns
- [ ] Safe motion with slow speed
- [ ] Return exact touch position
- [ ] Timeout và abort handling

---

#### P4-20: Touch Offset Calibration
**Mô tả:** Hiệu chuẩn offset giữa wire tip và TCP.

**Nguyên lý:**
```
Wire tip không trùng với TCP (laser focus point).
Cần tìm offset vector từ TCP → Wire tip.

Phương pháp:
1. Touch vật chuẩn tại nhiều hướng khác nhau
2. Fit sphere qua các điểm touch
3. Tâm sphere = Vị trí wire tip trong tool frame
```

**DoD:**
- [ ] Calibrate offset trong 3D
- [ ] Accuracy < 0.3mm
- [ ] Auto-calculate từ multiple touches
- [ ] Save offset với TCP data

---

### 3.6 Nhóm F: Vision UI (P4-21 → P4-23)

#### P4-21: Point Cloud Visualization (Helix Toolkit)
**Mô tả:** Hiển thị 3D point cloud real-time.

**Features:**
- Render point cloud với color-coded intensity
- Zoom/Pan/Rotate controls
- Highlight detected seam line
- Show robot và sensor positions

**Implementation:**
```csharp
public class PointCloudVisualizer : HelixViewport3D
{
    // Point cloud data
    public void UpdatePointCloud(Point3DCollection points,
                                  Color4Collection colors);

    // Seam path overlay
    public void ShowSeamPath(List<Point3D> path, Color color);

    // Coordinate frames
    public void ShowFrame(Matrix3D transform, string name);

    // Settings
    public double PointSize { get; set; } = 2.0;
    public bool ShowAxes { get; set; } = true;
    public bool AutoRotate { get; set; } = false;
}
```

**DoD:**
- [ ] Render 100k+ points ở 30fps
- [ ] Color by intensity/height
- [ ] Interactive camera controls
- [ ] Export to PLY/XYZ format

---

#### P4-22: Laser Profile View (2D)
**Mô tả:** Hiển thị profile 2D real-time với detected features.

**UI Layout:**
```
┌─────────────────────────────────────────────────────────────┐
│  LASER PROFILE VIEW                            [Freeze][Rec]│
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  Z (mm)                                                      │
│  │     ╱╲                                                   │
│  │    ╱  ╲                                                  │
│  │   ╱    ╲          ╱╲                                     │
│  │  ╱      ╲________╱  ╲                                    │
│  │ ╱                    ╲                                   │
│  │╱________________________╲___________________________      │
│  └─────────────────────────────────────────────────── X (mm)│
│                                                              │
│  ● Root Point: X=50.2, Z=-5.3                               │
│  ● Groove Angle: 60°                                        │
│  ● Confidence: 98%                                          │
└─────────────────────────────────────────────────────────────┘
```

**DoD:**
- [ ] Real-time profile display ở 100Hz
- [ ] Overlay detected features
- [ ] Zoom và scroll
- [ ] Record và playback

---

#### P4-23: Tracking Monitor Dashboard
**Mô tả:** Dashboard giám sát seam tracking real-time.

**UI Layout:**
```
┌─────────────────────────────────────────────────────────────┐
│  SEAM TRACKING MONITOR                         [TRACKING ON]│
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌───────────────────┐  ┌───────────────────────────────┐   │
│  │   CORRECTION      │  │   ERROR HISTORY               │   │
│  │                   │  │                               │   │
│  │   X: +0.23 mm     │  │   ___/\___/\___/\___         │   │
│  │   Y: -0.15 mm     │  │                               │   │
│  │   Z: +0.02 mm     │  │   X ────                      │   │
│  │                   │  │   Y ----                      │   │
│  │   [■■■■■□□□□□]    │  │   Z ....                      │   │
│  │     0    ±5mm     │  │                               │   │
│  └───────────────────┘  └───────────────────────────────┘   │
│                                                              │
│  ┌───────────────────┐  ┌───────────────────────────────┐   │
│  │   STATUS          │  │   CONFIGURATION               │   │
│  │                   │  │                               │   │
│  │   State: TRACKING │  │   Max Correction: 5.0 mm      │   │
│  │   Confidence: 95% │  │   Look-ahead: 50 mm           │   │
│  │   Lost frames: 0  │  │   Filter: Kalman              │   │
│  │   Latency: 28ms   │  │                               │   │
│  └───────────────────┘  └───────────────────────────────┘   │
│                                                              │
│  [ENABLE]  [DISABLE]  [RESET]  [CONFIG...]                  │
└─────────────────────────────────────────────────────────────┘
```

**DoD:**
- [ ] Real-time correction display
- [ ] Error history graph (last 10 seconds)
- [ ] Status indicators
- [ ] Configuration controls

---

## 4. HARDWARE SPECIFICATIONS

### 4.1 Recommended Laser Profilers

| Model | Type | Wavelength | Resolution | FOV | Interface | Price Range |
|-------|------|------------|------------|-----|-----------|-------------|
| Hikrobot MV-DP2025 | Blue Laser | 405nm | 0.05mm | 100mm | GigE | $2,000-3,000 |
| Mech-Mind LNX-7500 | Blue Laser | 450nm | 0.03mm | 150mm | GigE | $3,000-4,000 |
| Riftek RF627Weld | Blue Laser | 450nm | 0.02mm | 100mm | Ethernet | $4,000-5,000 |
| Keyence LJ-X8000 | Blue Laser | 405nm | 0.01mm | 120mm | GigE | $8,000-12,000 |

### 4.2 Sensor Mounting

```
          ┌────────────────┐
          │   Robot Flange │
          └───────┬────────┘
                  │
          ┌───────┴────────┐
          │  Sensor Mount  │
          │   (Aluminum)   │
          └───────┬────────┘
                  │
          ┌───────┴────────┐
          │ Laser Profiler │
          │                │
          │    ◄───────────┼── Look-ahead distance
          │                │   (typically 50-100mm)
          └────────────────┘
                  │
                  ▼
          Weld Direction ────────────────────▶
```

### 4.3 Touch Sensing Hardware

**Components:**
- 24V DC power supply
- Current limiting resistor (1k-10k ohm)
- Optocoupler for isolation
- Signal conditioning circuit

**Schematic:**
```
+24V ───┬─── 1kΩ ───┬─── Wire Tip ───○ Workpiece (Ground)
        │           │
        │       ┌───┴───┐
        │       │  Opto │
        │       │ coupler│
        │       └───┬───┘
        │           │
GND ────┴───────────┴─── DI to Controller
```

---

## 5. ALGORITHM SPECIFICATIONS

### 5.1 RANSAC Parameters

| Parameter | V-Groove | Fillet | Description |
|-----------|----------|--------|-------------|
| distanceThreshold | 0.5mm | 1.0mm | Max distance from model |
| maxIterations | 1000 | 500 | RANSAC iterations |
| minInlierRatio | 0.7 | 0.6 | Min inliers percentage |
| angleConstraint | 30-90° | 45-135° | Valid groove angle range |

### 5.2 Kalman Filter Parameters

```cpp
// Process noise (Q matrix diagonal)
double pos_noise = 0.01;   // mm²/step
double vel_noise = 0.1;    // (mm/s)²/step

// Measurement noise (R matrix diagonal)
double meas_noise = 0.5;   // mm²

// Initial covariance (P matrix diagonal)
double init_pos_var = 10.0;  // mm²
double init_vel_var = 100.0; // (mm/s)²
```

### 5.3 Euler Angle Conventions

| Robot Brand | Convention | Order | Notes |
|-------------|------------|-------|-------|
| Universal Robots | Axis-Angle | - | Rotation vector (Rx, Ry, Rz) |
| KUKA | ZYX Euler | Z→Y'→X'' | A, B, C angles |
| Fanuc | XYZ Euler | X→Y'→Z'' | W, P, R angles |
| ABB | Quaternion | - | q0, qx, qy, qz |

**Conversion (Rotation Matrix → ZYX Euler):**
```cpp
void rotationMatrixToZYXEuler(const Eigen::Matrix3d& R,
                               double& rz, double& ry, double& rx) {
    // Check for gimbal lock
    if (std::abs(R(2,0)) > 0.9999) {
        // Gimbal lock
        rz = 0;
        ry = (R(2,0) < 0) ? M_PI/2 : -M_PI/2;
        rx = std::atan2(-R(0,1), R(1,1));
    } else {
        ry = -std::asin(R(2,0));
        rx = std::atan2(R(2,1)/std::cos(ry), R(2,2)/std::cos(ry));
        rz = std::atan2(R(1,0)/std::cos(ry), R(0,0)/std::cos(ry));
    }
}
```

---

## 6. TESTING & DEFINITION OF DONE

### 6.1 Unit Test Coverage

| Module | Test File | Coverage Target |
|--------|-----------|-----------------|
| NoiseFilter | test_NoiseFilter.cpp | 90% |
| FeatureExtractor | test_FeatureExtractor.cpp | 90% |
| SeamTracker | test_SeamTracker.cpp | 85% |
| Calibration | test_Calibration.cpp | 95% |
| TouchSense | test_TouchSense.cpp | 90% |

### 6.2 Integration Tests

| Test Case | Description | Pass Criteria |
|-----------|-------------|---------------|
| IT-V-01 | Sensor connection | Connect & stream data |
| IT-V-02 | V-Groove detection | Detect groove ±0.5mm |
| IT-V-03 | Real-time tracking | Track at 100Hz, <50ms latency |
| IT-V-04 | Hand-Eye calibration | Error <1mm, <0.5° |
| IT-V-05 | Touch sensing | Detect touch ±0.3mm |

### 6.3 Hardware-in-Loop Tests

| Test Case | Description | Equipment |
|-----------|-------------|-----------|
| HIL-V-01 | Sensor accuracy | Calibration block |
| HIL-V-02 | Tracking accuracy | Moving target with encoder |
| HIL-V-03 | Full seam tracking | Real weld joint |

### 6.4 Phase 4 Acceptance Criteria

- [ ] Robot scans workpiece and detects V-Groove/Fillet seam path
- [ ] Seam Tracking compensates ±5mm deviation with <50ms latency
- [ ] Touch Sensing detects contact point with ±0.5mm accuracy
- [ ] TCP Calibration achieves <0.1mm error
- [ ] Hand-Eye Calibration achieves <1mm translation, <0.5° rotation error
- [ ] All unit tests pass (>85% coverage)
- [ ] Integration tests pass
- [ ] Documentation complete

---

## 7. DEPENDENCIES & PREREQUISITES

### 7.1 Phase 3 Prerequisites
Trước khi bắt đầu Phase 4, các module sau phải hoàn thành:
- [P3-01] Digital Output Interface
- [P3-02] Digital Input Interface
- [P3-07] WeldingSequencer (for coordination)

### 7.2 Hardware Requirements
- Laser Profiler (Blue Laser, GigE Vision)
- Encoder output từ robot (for hardware trigger)
- Touch sensing circuit
- Dedicated NIC card (Jumbo Frames support)

### 7.3 Software Dependencies

| Library | Version | Purpose | License |
|---------|---------|---------|---------|
| PCL | 1.12+ | Point cloud processing | BSD |
| Eigen3 | 3.4+ | Linear algebra | MPL2 |
| OpenCV | 4.5+ | Image processing | Apache 2.0 |
| Hikrobot MVS | 4.0+ | Sensor SDK | Proprietary |

### 7.4 Build Configuration

**CMake for Native DLL:**
```cmake
cmake_minimum_required(VERSION 3.16)
project(VisionCore)

set(CMAKE_CXX_STANDARD 17)

find_package(PCL 1.12 REQUIRED)
find_package(Eigen3 3.4 REQUIRED)
find_package(OpenCV 4.5 REQUIRED)

add_library(VisionCore SHARED
    VisionCore.cpp
    PCLWrapper.cpp
    FeatureExtractor.cpp
    NoiseFilter.cpp
)

target_link_libraries(VisionCore
    ${PCL_LIBRARIES}
    Eigen3::Eigen
    ${OpenCV_LIBS}
)
```

---

## 8. OPEN SOURCE REFERENCES

### 8.1 Point Cloud Processing
- [PCL Documentation](https://pointclouds.org/documentation/)
- [Open3D](https://www.open3d.org/)
- [indraneelpatil/3D-Weld-Seam-Tracking-using-PCL](https://github.com/indraneelpatil/3D-Weld-Seam-Tracking-using-PCL)
- [thillRobot/seam_detection](https://github.com/thillRobot/seam_detection)

### 8.2 Calibration
- [OpenCV Camera Calibration](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)
- [easy_handeye (ROS)](https://github.com/IFL-CAMP/easy_handeye)

### 8.3 Line Extraction
- [wakkaliu/Steger-Centerline](https://github.com/wakkaliu/Steger-Centerline)
- [kam3k/laser_line_extraction](https://github.com/kam3k/laser_line_extraction)

### 8.4 Sensor SDKs
- [RIFTEK-LLC/RF62X-SDK](https://github.com/RIFTEK-LLC/RF62X-SDK)
- [MechMindRobotics/mecheye_csharp_samples](https://github.com/MechMindRobotics/mecheye_csharp_samples)

---

## 9. REVISION HISTORY

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 01/02/2026 | - | Initial draft |

---

*Document generated following project roadmap structure.*
