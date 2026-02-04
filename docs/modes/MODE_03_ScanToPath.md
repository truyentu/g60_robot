# MODE_03: Scan-to-Path Mode

| Metadata      | Value                           |
|---------------|---------------------------------|
| Module ID     | MODE_03                         |
| Priority      | P2 (Advanced Mode)              |
| Status        | DRAFT                           |
| Dependencies  | CORE_02, CORE_03, CORE_07       |

---

## 1. Tổng quan

### 1.1 Mục đích

Scan-to-Path mode cho phép robot quét 3D bề mặt workpiece và tự động tạo đường chạy (toolpath) dựa trên dữ liệu scan. Mode này hỗ trợ adaptive welding, seam tracking, và quality inspection.

### 1.2 Use Cases

| Use Case | Mô tả |
|----------|-------|
| **Seam Finding** | Tự động tìm mối hàn từ scan 3D |
| **Adaptive Welding** | Điều chỉnh đường hàn theo hình dạng thực tế |
| **Gap Compensation** | Bù khe hở dựa trên scan data |
| **Quality Inspection** | So sánh scan với CAD để kiểm tra |
| **Reverse Engineering** | Tạo model 3D từ chi tiết thực |

### 1.3 Kiến trúc hệ thống

```
┌─────────────────────────────────────────────────────────────────┐
│                         HMI Layer (C#)                          │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐          │
│  │ ScanView     │  │ PathGenView  │  │ CompareView  │          │
│  │ (Point Cloud)│  │ (Toolpath)   │  │ (CAD vs Scan)│          │
│  └──────────────┘  └──────────────┘  └──────────────┘          │
└───────────────────────────┬─────────────────────────────────────┘
                            │ ZeroMQ PUB-SUB (Point Cloud Stream)
┌───────────────────────────┴─────────────────────────────────────┐
│                      Core Logic (C++)                           │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐          │
│  │ ScanManager  │  │ PathGenerator│  │ SeamDetector │          │
│  └──────────────┘  └──────────────┘  └──────────────┘          │
└───────────────────────────┬─────────────────────────────────────┘
                            │ ZeroMQ REQ-REP
┌───────────────────────────┴─────────────────────────────────────┐
│                  3D Processing (Python/C++)                     │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐          │
│  │ Open3D       │  │ PCL          │  │ CGAL         │          │
│  │ Processing   │  │ Filters      │  │ Mesh Ops     │          │
│  └──────────────┘  └──────────────┘  └──────────────┘          │
└───────────────────────────┬─────────────────────────────────────┘
                            │ USB3 / Ethernet
┌───────────────────────────┴─────────────────────────────────────┐
│                      3D Scanner Hardware                        │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐          │
│  │ Intel        │  │ Zivid        │  │ Laser Line   │          │
│  │ RealSense    │  │ One+         │  │ Scanner      │          │
│  └──────────────┘  └──────────────┘  └──────────────┘          │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. 3D Scanner Integration

### 2.1 Supported Scanners

| Scanner | Type | Resolution | Range | Interface |
|---------|------|------------|-------|-----------|
| **Intel RealSense D435** | Stereo Depth | 1280x720 | 0.3-3m | USB3 |
| **Intel RealSense L515** | LiDAR | 1024x768 | 0.25-9m | USB3 |
| **Zivid One+** | Structured Light | 2048x2048 | 0.3-1.2m | Ethernet |
| **Keyence LJ-X8000** | Laser Line | 3200 pts/line | 24-300mm | Ethernet |

### 2.2 Scanner Abstraction Layer

```cpp
// ScannerInterface.h
#pragma once
#include <memory>
#include <vector>
#include <Eigen/Dense>

namespace Scan {

struct Point3D {
    float x, y, z;
    float nx, ny, nz;     // Normal
    uint8_t r, g, b;      // Color (optional)
    float intensity;       // Reflectance
};

struct PointCloud {
    std::vector<Point3D> points;
    uint64_t timestamp_us;
    int frame_id;

    // Metadata
    std::string scanner_id;
    Eigen::Matrix4d sensor_pose;  // T_base_to_sensor at capture time

    // Statistics
    size_t size() const { return points.size(); }
    bool empty() const { return points.empty(); }

    // Bounding box
    Eigen::Vector3f min_bound;
    Eigen::Vector3f max_bound;
};

enum class ScannerType {
    REALSENSE_D435,
    REALSENSE_L515,
    ZIVID_ONE_PLUS,
    KEYENCE_LJ_X8000,
    SIMULATION
};

struct ScannerConfig {
    ScannerType type;
    std::string device_id;

    // Resolution
    int width = 1280;
    int height = 720;

    // Depth settings
    float min_depth_m = 0.3f;
    float max_depth_m = 1.5f;

    // Filter settings
    bool enable_decimation = true;
    int decimation_factor = 2;
    bool enable_spatial_filter = true;
    bool enable_temporal_filter = true;

    // Exposure
    int exposure_us = 8500;
    int laser_power = 150;  // For structured light
};

class IScanner {
public:
    virtual ~IScanner() = default;

    // Lifecycle
    virtual bool connect() = 0;
    virtual void disconnect() = 0;
    virtual bool isConnected() const = 0;

    // Configuration
    virtual void configure(const ScannerConfig& config) = 0;
    virtual ScannerConfig getConfig() const = 0;

    // Capture
    virtual std::shared_ptr<PointCloud> capture() = 0;
    virtual void startStreaming(
        std::function<void(std::shared_ptr<PointCloud>)> callback) = 0;
    virtual void stopStreaming() = 0;

    // Calibration
    virtual Eigen::Matrix4d getExtrinsics() const = 0;
    virtual void setExtrinsics(const Eigen::Matrix4d& T) = 0;
};

// Factory
std::unique_ptr<IScanner> createScanner(ScannerType type);

} // namespace Scan
```

### 2.3 RealSense Implementation

```cpp
// RealSenseScanner.cpp
#include "RealSenseScanner.h"
#include <librealsense2/rs.hpp>

namespace Scan {

class RealSenseScanner : public IScanner {
public:
    bool connect() override {
        try {
            rs2::context ctx;
            auto devices = ctx.query_devices();
            if (devices.size() == 0) {
                return false;
            }

            // Configure pipeline
            rs2::config cfg;
            cfg.enable_stream(RS2_STREAM_DEPTH,
                config_.width, config_.height,
                RS2_FORMAT_Z16, 30);
            cfg.enable_stream(RS2_STREAM_COLOR,
                config_.width, config_.height,
                RS2_FORMAT_RGB8, 30);

            // Start pipeline
            pipe_ = std::make_unique<rs2::pipeline>();
            profile_ = pipe_->start(cfg);

            // Get intrinsics
            auto depth_stream = profile_.get_stream(RS2_STREAM_DEPTH)
                                        .as<rs2::video_stream_profile>();
            intrinsics_ = depth_stream.get_intrinsics();

            connected_ = true;
            return true;
        }
        catch (const rs2::error& e) {
            spdlog::error("RealSense error: {}", e.what());
            return false;
        }
    }

    std::shared_ptr<PointCloud> capture() override {
        if (!connected_) return nullptr;

        rs2::frameset frames = pipe_->wait_for_frames();

        // Apply filters
        frames = align_to_color_.process(frames);

        if (config_.enable_decimation) {
            frames = decimation_filter_.process(frames);
        }
        if (config_.enable_spatial_filter) {
            frames = spatial_filter_.process(frames);
        }
        if (config_.enable_temporal_filter) {
            frames = temporal_filter_.process(frames);
        }

        // Get depth and color
        auto depth = frames.get_depth_frame();
        auto color = frames.get_color_frame();

        // Generate point cloud
        rs2::pointcloud pc;
        pc.map_to(color);
        rs2::points points = pc.calculate(depth);

        return convertToPointCloud(points, color);
    }

private:
    std::shared_ptr<PointCloud> convertToPointCloud(
        const rs2::points& rs_points,
        const rs2::video_frame& color) {

        auto cloud = std::make_shared<PointCloud>();
        cloud->timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();

        auto vertices = rs_points.get_vertices();
        auto tex_coords = rs_points.get_texture_coordinates();
        const uint8_t* color_data = (const uint8_t*)color.get_data();
        int color_stride = color.get_stride_in_bytes();

        cloud->points.reserve(rs_points.size());
        cloud->min_bound = Eigen::Vector3f::Constant(FLT_MAX);
        cloud->max_bound = Eigen::Vector3f::Constant(-FLT_MAX);

        for (size_t i = 0; i < rs_points.size(); i++) {
            if (vertices[i].z == 0) continue;
            if (vertices[i].z < config_.min_depth_m) continue;
            if (vertices[i].z > config_.max_depth_m) continue;

            Point3D pt;
            pt.x = vertices[i].x * 1000.0f;  // Convert to mm
            pt.y = vertices[i].y * 1000.0f;
            pt.z = vertices[i].z * 1000.0f;

            // Get color from texture coordinates
            int u = std::clamp((int)(tex_coords[i].u * color.get_width()),
                              0, color.get_width() - 1);
            int v = std::clamp((int)(tex_coords[i].v * color.get_height()),
                              0, color.get_height() - 1);
            const uint8_t* pixel = color_data + v * color_stride + u * 3;
            pt.r = pixel[0];
            pt.g = pixel[1];
            pt.b = pixel[2];

            cloud->points.push_back(pt);

            // Update bounds
            cloud->min_bound = cloud->min_bound.cwiseMin(
                Eigen::Vector3f(pt.x, pt.y, pt.z));
            cloud->max_bound = cloud->max_bound.cwiseMax(
                Eigen::Vector3f(pt.x, pt.y, pt.z));
        }

        return cloud;
    }

    std::unique_ptr<rs2::pipeline> pipe_;
    rs2::pipeline_profile profile_;
    rs2_intrinsics intrinsics_;

    rs2::align align_to_color_{RS2_STREAM_COLOR};
    rs2::decimation_filter decimation_filter_;
    rs2::spatial_filter spatial_filter_;
    rs2::temporal_filter temporal_filter_;

    ScannerConfig config_;
    bool connected_ = false;
};

} // namespace Scan
```

---

## 3. Point Cloud Processing

### 3.1 Processing Pipeline

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│  Raw Point  │───▶│  Filter &   │───▶│   Normal    │───▶│  Transform  │
│    Cloud    │    │  Denoise    │    │  Estimation │    │  to Robot   │
└─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘
                                                                │
                                                                ▼
┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│  Toolpath   │◀───│  Feature    │◀───│   Surface   │◀───│ Registration│
│  Generation │    │  Extraction │    │  Reconstruct│    │  (ICP)      │
└─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘
```

### 3.2 Point Cloud Processor

```cpp
// PointCloudProcessor.h
#pragma once
#include "ScannerInterface.h"
#include <open3d/Open3D.h>

namespace Scan {

struct ProcessingConfig {
    // Voxel downsampling
    double voxel_size_mm = 1.0;

    // Statistical outlier removal
    int sor_neighbors = 20;
    double sor_std_ratio = 2.0;

    // Normal estimation
    int normal_neighbors = 30;

    // Plane segmentation (for workpiece extraction)
    double plane_distance_threshold_mm = 5.0;
    int ransac_n = 3;
    int num_iterations = 1000;

    // Clustering
    double cluster_eps_mm = 10.0;
    int min_cluster_points = 100;
};

class PointCloudProcessor {
public:
    explicit PointCloudProcessor(const ProcessingConfig& config = {});

    // Main processing pipeline
    std::shared_ptr<PointCloud> process(
        const std::shared_ptr<PointCloud>& input);

    // Individual operations
    std::shared_ptr<PointCloud> voxelDownsample(
        const std::shared_ptr<PointCloud>& cloud);

    std::shared_ptr<PointCloud> removeOutliers(
        const std::shared_ptr<PointCloud>& cloud);

    void estimateNormals(std::shared_ptr<PointCloud>& cloud);

    std::shared_ptr<PointCloud> transformToRobotFrame(
        const std::shared_ptr<PointCloud>& cloud,
        const Eigen::Matrix4d& T_base_to_sensor);

    // Segmentation
    std::vector<std::shared_ptr<PointCloud>> segmentPlanes(
        const std::shared_ptr<PointCloud>& cloud,
        std::vector<Eigen::Vector4d>& plane_coefficients);

    std::vector<std::shared_ptr<PointCloud>> clusterDBSCAN(
        const std::shared_ptr<PointCloud>& cloud);

    // Registration
    Eigen::Matrix4d registerICP(
        const std::shared_ptr<PointCloud>& source,
        const std::shared_ptr<PointCloud>& target,
        const Eigen::Matrix4d& initial_guess = Eigen::Matrix4d::Identity());

    // Surface reconstruction
    std::shared_ptr<open3d::geometry::TriangleMesh> reconstructSurface(
        const std::shared_ptr<PointCloud>& cloud);

private:
    ProcessingConfig config_;

    // Convert to/from Open3D
    std::shared_ptr<open3d::geometry::PointCloud> toOpen3D(
        const std::shared_ptr<PointCloud>& cloud);
    std::shared_ptr<PointCloud> fromOpen3D(
        const std::shared_ptr<open3d::geometry::PointCloud>& o3d_cloud);
};

} // namespace Scan
```

### 3.3 Processing Implementation

```cpp
// PointCloudProcessor.cpp
#include "PointCloudProcessor.h"

namespace Scan {

std::shared_ptr<PointCloud> PointCloudProcessor::process(
    const std::shared_ptr<PointCloud>& input) {

    if (!input || input->empty()) {
        return nullptr;
    }

    auto cloud = input;

    // 1. Voxel downsample
    cloud = voxelDownsample(cloud);
    spdlog::debug("After voxel downsample: {} points", cloud->size());

    // 2. Remove outliers
    cloud = removeOutliers(cloud);
    spdlog::debug("After outlier removal: {} points", cloud->size());

    // 3. Estimate normals
    estimateNormals(cloud);

    return cloud;
}

std::shared_ptr<PointCloud> PointCloudProcessor::voxelDownsample(
    const std::shared_ptr<PointCloud>& cloud) {

    auto o3d_cloud = toOpen3D(cloud);
    auto downsampled = o3d_cloud->VoxelDownSample(config_.voxel_size_mm);
    return fromOpen3D(downsampled);
}

std::shared_ptr<PointCloud> PointCloudProcessor::removeOutliers(
    const std::shared_ptr<PointCloud>& cloud) {

    auto o3d_cloud = toOpen3D(cloud);
    auto [filtered, indices] = o3d_cloud->RemoveStatisticalOutliers(
        config_.sor_neighbors, config_.sor_std_ratio);
    return fromOpen3D(filtered);
}

void PointCloudProcessor::estimateNormals(std::shared_ptr<PointCloud>& cloud) {
    auto o3d_cloud = toOpen3D(cloud);

    o3d_cloud->EstimateNormals(
        open3d::geometry::KDTreeSearchParamHybrid(
            config_.voxel_size_mm * 2, config_.normal_neighbors));

    // Orient normals towards camera (assumes Z-up for robot)
    o3d_cloud->OrientNormalsTowardsCameraLocation(Eigen::Vector3d(0, 0, 1000));

    // Copy normals back
    const auto& normals = o3d_cloud->normals_;
    for (size_t i = 0; i < cloud->points.size() && i < normals.size(); i++) {
        cloud->points[i].nx = normals[i](0);
        cloud->points[i].ny = normals[i](1);
        cloud->points[i].nz = normals[i](2);
    }
}

Eigen::Matrix4d PointCloudProcessor::registerICP(
    const std::shared_ptr<PointCloud>& source,
    const std::shared_ptr<PointCloud>& target,
    const Eigen::Matrix4d& initial_guess) {

    auto o3d_source = toOpen3D(source);
    auto o3d_target = toOpen3D(target);

    // Point-to-plane ICP
    auto result = open3d::pipelines::registration::RegistrationICP(
        *o3d_source, *o3d_target,
        config_.voxel_size_mm * 2,  // Max correspondence distance
        initial_guess,
        open3d::pipelines::registration::TransformationEstimationPointToPlane());

    spdlog::info("ICP fitness: {:.4f}, RMSE: {:.4f}mm",
                result.fitness_, result.inlier_rmse_);

    return result.transformation_;
}

} // namespace Scan
```

### 3.4 Python Processing Service

```python
# scan_processing_server.py
import zmq
import numpy as np
import open3d as o3d
from dataclasses import dataclass
from typing import List, Tuple, Optional
import json

@dataclass
class ProcessingResult:
    points: np.ndarray          # Nx3
    normals: np.ndarray         # Nx3
    colors: np.ndarray          # Nx3 (0-1)
    features: dict              # Extracted features

class ScanProcessingServer:
    def __init__(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)

    def start(self, port: int = 5580):
        self.socket.bind(f"tcp://*:{port}")
        print(f"Scan processing server listening on port {port}")

        while True:
            message = self.socket.recv()

            # Deserialize point cloud
            cloud = self.deserialize_pointcloud(message)

            # Process
            result = self.process(cloud)

            # Send result
            response = self.serialize_result(result)
            self.socket.send(response)

    def process(self, pcd: o3d.geometry.PointCloud) -> ProcessingResult:
        # Downsample
        pcd_down = pcd.voxel_down_sample(voxel_size=1.0)

        # Remove outliers
        pcd_clean, _ = pcd_down.remove_statistical_outlier(
            nb_neighbors=20, std_ratio=2.0)

        # Estimate normals
        pcd_clean.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=5.0, max_nn=30))

        # Extract features
        features = self.extract_features(pcd_clean)

        return ProcessingResult(
            points=np.asarray(pcd_clean.points),
            normals=np.asarray(pcd_clean.normals),
            colors=np.asarray(pcd_clean.colors) if pcd_clean.has_colors() else None,
            features=features
        )

    def extract_features(self, pcd: o3d.geometry.PointCloud) -> dict:
        features = {}

        # Bounding box
        aabb = pcd.get_axis_aligned_bounding_box()
        features['bbox_min'] = aabb.min_bound.tolist()
        features['bbox_max'] = aabb.max_bound.tolist()
        features['bbox_center'] = aabb.get_center().tolist()

        # Plane detection (table/fixture)
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=5.0, ransac_n=3, num_iterations=1000)
        features['dominant_plane'] = plane_model.tolist()
        features['plane_inlier_count'] = len(inliers)

        # Workpiece (points above plane)
        plane_normal = np.array(plane_model[:3])
        plane_d = plane_model[3]
        points = np.asarray(pcd.points)
        distances = np.dot(points, plane_normal) + plane_d
        workpiece_mask = distances > 10.0  # 10mm above plane
        features['workpiece_point_count'] = int(np.sum(workpiece_mask))

        return features
```

---

## 4. Seam Detection

### 4.1 Seam Types

```
┌─────────────────────────────────────────────────────────────────┐
│                     WELD SEAM TYPES                             │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Butt Joint:           Lap Joint:           Fillet Joint:       │
│  ┌─────┐ ┌─────┐      ┌──────────┐         ┌─────┐              │
│  │     │ │     │      │          │         │     │              │
│  │     │▼│     │      │    ┌─────┘         │     │              │
│  └─────┘ └─────┘      └────┤               └─────┴─────┐        │
│     Seam line              │ Seam line           Seam  │        │
│                            │                     line  │        │
│                                                                 │
│  V-Groove:             Corner Joint:        Edge Joint:         │
│  ┌─────┐ ┌─────┐      ┌─────┐              ┌──────────┐         │
│  │    ╲│ │╱    │      │     │              │          │         │
│  │     ▼╳▼     │      │     └─────┐        └──────────┘         │
│  └─────┘ └─────┘      └─────┘     │        ┌──────────┐         │
│     V seam                  └─────┘        │          │         │
│                         Seam               └──────────┘         │
│                                             Edge seam           │
└─────────────────────────────────────────────────────────────────┘
```

### 4.2 Seam Detector

```cpp
// SeamDetector.h
#pragma once
#include "PointCloudProcessor.h"

namespace Scan {

enum class SeamType {
    BUTT,
    LAP,
    FILLET,
    V_GROOVE,
    CORNER,
    EDGE,
    UNKNOWN
};

struct SeamPoint {
    Eigen::Vector3d position;     // Point on seam (mm)
    Eigen::Vector3d direction;    // Tangent direction
    Eigen::Vector3d normal;       // Surface normal at seam
    Eigen::Vector3d approach;     // Torch approach vector
    double gap_width_mm = 0.0;    // Gap width (for butt joints)
    double angle_deg = 0.0;       // Joint angle
};

struct DetectedSeam {
    SeamType type = SeamType::UNKNOWN;
    std::vector<SeamPoint> points;  // Ordered seam points
    double total_length_mm = 0.0;
    double confidence = 0.0;

    // Fitted curve parameters (optional)
    bool has_fitted_curve = false;
    std::vector<double> curve_params;  // Polynomial or spline coefficients
};

struct SeamDetectionConfig {
    // Edge detection
    double edge_angle_threshold_deg = 30.0;
    double min_edge_length_mm = 20.0;

    // Gap detection
    double max_gap_width_mm = 10.0;
    double min_gap_width_mm = 0.5;

    // Seam sampling
    double sample_spacing_mm = 2.0;

    // Filtering
    int smoothing_window = 5;
    double outlier_threshold_mm = 3.0;
};

class SeamDetector {
public:
    explicit SeamDetector(const SeamDetectionConfig& config = {});

    // Detect seams in point cloud
    std::vector<DetectedSeam> detectSeams(
        const std::shared_ptr<PointCloud>& cloud);

    // Detect specific seam type
    std::optional<DetectedSeam> detectButtJoint(
        const std::shared_ptr<PointCloud>& cloud);

    std::optional<DetectedSeam> detectFilletJoint(
        const std::shared_ptr<PointCloud>& cloud);

    std::optional<DetectedSeam> detectLapJoint(
        const std::shared_ptr<PointCloud>& cloud);

    // Refine seam with higher resolution scan
    DetectedSeam refineSeam(
        const DetectedSeam& coarse_seam,
        const std::shared_ptr<PointCloud>& high_res_cloud);

    // Fit curve to seam points
    void fitCurve(DetectedSeam& seam, int polynomial_order = 3);

private:
    // Edge detection
    std::vector<Eigen::Vector3d> detectEdges(
        const std::shared_ptr<PointCloud>& cloud);

    // Curvature-based feature detection
    std::vector<int> findHighCurvaturePoints(
        const std::shared_ptr<PointCloud>& cloud);

    // Gap detection (for butt joints)
    std::vector<std::pair<Eigen::Vector3d, double>> detectGaps(
        const std::shared_ptr<PointCloud>& cloud);

    // Intersection detection (for fillet/corner)
    std::vector<Eigen::Vector3d> detectPlaneIntersections(
        const std::vector<Eigen::Vector4d>& planes);

    // Compute approach vector
    Eigen::Vector3d computeApproachVector(
        const SeamPoint& point,
        SeamType type);

    SeamDetectionConfig config_;
    PointCloudProcessor processor_;
};

} // namespace Scan
```

### 4.3 Seam Detection Implementation

```cpp
// SeamDetector.cpp
#include "SeamDetector.h"

namespace Scan {

std::vector<DetectedSeam> SeamDetector::detectSeams(
    const std::shared_ptr<PointCloud>& cloud) {

    std::vector<DetectedSeam> seams;

    // Try different seam types
    if (auto butt = detectButtJoint(cloud)) {
        seams.push_back(*butt);
    }

    if (auto fillet = detectFilletJoint(cloud)) {
        seams.push_back(*fillet);
    }

    if (auto lap = detectLapJoint(cloud)) {
        seams.push_back(*lap);
    }

    // Sort by confidence
    std::sort(seams.begin(), seams.end(),
        [](const DetectedSeam& a, const DetectedSeam& b) {
            return a.confidence > b.confidence;
        });

    return seams;
}

std::optional<DetectedSeam> SeamDetector::detectFilletJoint(
    const std::shared_ptr<PointCloud>& cloud) {

    // 1. Segment planes
    std::vector<Eigen::Vector4d> planes;
    auto plane_clouds = processor_.segmentPlanes(cloud, planes);

    if (planes.size() < 2) {
        return std::nullopt;  // Need at least 2 planes for fillet
    }

    // 2. Find plane intersections
    DetectedSeam seam;
    seam.type = SeamType::FILLET;

    for (size_t i = 0; i < planes.size() - 1; i++) {
        for (size_t j = i + 1; j < planes.size(); j++) {
            // Compute angle between planes
            Eigen::Vector3d n1 = planes[i].head<3>();
            Eigen::Vector3d n2 = planes[j].head<3>();
            double angle = std::acos(std::abs(n1.dot(n2))) * 180.0 / M_PI;

            // Fillet joints typically 60-120 degrees
            if (angle < 60 || angle > 120) continue;

            // Find intersection line
            Eigen::Vector3d line_dir = n1.cross(n2).normalized();

            // Find a point on the intersection
            // Solve: n1·p = -d1 and n2·p = -d2
            Eigen::Matrix<double, 2, 3> A;
            A.row(0) = n1.transpose();
            A.row(1) = n2.transpose();
            Eigen::Vector2d b(-planes[i](3), -planes[j](3));

            // Use pseudoinverse
            Eigen::Vector3d point_on_line =
                A.completeOrthogonalDecomposition().solve(b);

            // Sample points along intersection within cloud bounds
            double start_t = -500, end_t = 500;  // mm
            for (double t = start_t; t <= end_t; t += config_.sample_spacing_mm) {
                Eigen::Vector3d p = point_on_line + t * line_dir;

                // Check if point is near actual cloud points
                // (simplified - in practice use KD-tree)
                bool near_cloud = false;
                for (const auto& cp : cloud->points) {
                    Eigen::Vector3d cloud_pt(cp.x, cp.y, cp.z);
                    if ((cloud_pt - p).norm() < 10.0) {
                        near_cloud = true;
                        break;
                    }
                }

                if (near_cloud) {
                    SeamPoint sp;
                    sp.position = p;
                    sp.direction = line_dir;
                    sp.normal = (n1 + n2).normalized();
                    sp.angle_deg = angle;
                    sp.approach = computeApproachVector(sp, SeamType::FILLET);
                    seam.points.push_back(sp);
                }
            }
        }
    }

    if (seam.points.size() < 10) {
        return std::nullopt;
    }

    // Compute total length
    seam.total_length_mm = 0;
    for (size_t i = 1; i < seam.points.size(); i++) {
        seam.total_length_mm +=
            (seam.points[i].position - seam.points[i-1].position).norm();
    }

    seam.confidence = std::min(1.0, seam.points.size() / 100.0);

    return seam;
}

Eigen::Vector3d SeamDetector::computeApproachVector(
    const SeamPoint& point,
    SeamType type) {

    switch (type) {
        case SeamType::FILLET: {
            // Approach at 45 degrees to both surfaces
            // Assuming normal points into the corner
            Eigen::Vector3d up(0, 0, 1);  // Gravity direction
            Eigen::Vector3d approach = point.normal;

            // Ensure approach is roughly downward for welding
            if (approach.dot(up) > 0) {
                approach = -approach;
            }

            return approach.normalized();
        }

        case SeamType::BUTT: {
            // Approach perpendicular to seam, from above
            Eigen::Vector3d up(0, 0, 1);
            return -up;  // Straight down
        }

        case SeamType::LAP: {
            // Approach at angle to upper plate
            Eigen::Vector3d approach = point.normal;
            Eigen::Vector3d horizontal = point.direction.cross(
                Eigen::Vector3d(0, 0, 1)).normalized();

            // 45 degree angle
            return (approach + horizontal).normalized();
        }

        default:
            return -Eigen::Vector3d::UnitZ();
    }
}

} // namespace Scan
```

---

## 5. Path Generation

### 5.1 Toolpath Types

```
┌─────────────────────────────────────────────────────────────────┐
│                    TOOLPATH PATTERNS                            │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Single Pass:          Multi-Pass:           Weave Path:        │
│  ─────────────▶        ─────────────▶        /\/\/\/\/\/\▶      │
│                        ─────────────▶                           │
│                        ─────────────▶                           │
│                                                                 │
│  Stitch Pattern:       Skip Pattern:         Tack Pattern:      │
│  ───── ───── ─────▶   ──────   ──────▶      ● ● ● ● ● ●▶       │
│                                                                 │
│  Spiral In:            Spiral Out:           Raster:            │
│  ┌───────────┐         ↓                    ─────────────▶      │
│  │ ┌───────┐ │        ───────────┐          ◀─────────────      │
│  │ │ ┌───┐ │ │        │ ┌─────┐ │          ─────────────▶      │
│  │ │ │ ● │ │ │        │ │  ●  │ │          ◀─────────────      │
│  │ │ └───┘ │ │        │ └─────┘ │                               │
│  │ └───────┘ │        └─────────┘                               │
│  └───────────┘                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 5.2 Path Generator

```cpp
// PathGenerator.h
#pragma once
#include "SeamDetector.h"
#include "../Trajectory/TrajectoryPlanner.h"

namespace Scan {

enum class PathType {
    SINGLE_PASS,
    MULTI_PASS,
    WEAVE,
    STITCH,
    SKIP,
    TACK
};

struct PathConfig {
    PathType type = PathType::SINGLE_PASS;

    // Speeds
    double travel_speed_mm_s = 500.0;
    double weld_speed_mm_s = 10.0;
    double approach_speed_mm_s = 100.0;

    // Distances
    double approach_distance_mm = 30.0;
    double retract_distance_mm = 30.0;
    double standoff_mm = 15.0;  // Torch to work distance

    // Multi-pass
    int num_passes = 1;
    double pass_offset_mm = 3.0;  // Offset between passes

    // Weaving
    bool enable_weave = false;
    double weave_width_mm = 5.0;
    double weave_frequency_hz = 2.0;

    // Stitch/Skip
    double stitch_length_mm = 50.0;
    double skip_length_mm = 20.0;

    // Tack
    int num_tacks = 5;
    double tack_dwell_s = 1.0;
};

struct Waypoint {
    Motion::CartesianPose pose;
    double speed_mm_s;
    bool is_weld_on;           // Welding active
    bool is_weave_on;          // Weaving active
    double dwell_s = 0.0;      // Dwell time at point

    // For visualization
    enum class Type { APPROACH, WELD, RETRACT, TRAVEL } type;
};

struct GeneratedPath {
    std::vector<Waypoint> waypoints;
    double total_length_mm = 0.0;
    double weld_length_mm = 0.0;
    double travel_length_mm = 0.0;

    // Estimated times
    double estimated_weld_time_s = 0.0;
    double estimated_total_time_s = 0.0;

    // Source seam info
    SeamType seam_type;
    std::string description;
};

class PathGenerator {
public:
    explicit PathGenerator(const PathConfig& config = {});

    // Generate path from seam
    GeneratedPath generateFromSeam(const DetectedSeam& seam);

    // Generate path from point list
    GeneratedPath generateFromPoints(
        const std::vector<Eigen::Vector3d>& points,
        const std::vector<Eigen::Vector3d>& normals);

    // Path optimization
    void optimizePath(GeneratedPath& path);

    // Convert to trajectory
    std::vector<Motion::CartesianPose> toTrajectory(
        const GeneratedPath& path);

    // Export
    std::string toGcode(const GeneratedPath& path);
    void toJSON(const GeneratedPath& path, const std::string& filename);

private:
    Motion::CartesianPose seamPointToPose(
        const SeamPoint& sp,
        double standoff_mm);

    void addApproach(GeneratedPath& path, const SeamPoint& first_point);
    void addRetract(GeneratedPath& path, const SeamPoint& last_point);
    void addWeldPath(GeneratedPath& path, const DetectedSeam& seam);
    void addTackPath(GeneratedPath& path, const DetectedSeam& seam);
    void addStitchPath(GeneratedPath& path, const DetectedSeam& seam);

    PathConfig config_;
};

} // namespace Scan
```

### 5.3 Path Generation Implementation

```cpp
// PathGenerator.cpp
#include "PathGenerator.h"

namespace Scan {

GeneratedPath PathGenerator::generateFromSeam(const DetectedSeam& seam) {
    GeneratedPath path;
    path.seam_type = seam.type;

    if (seam.points.empty()) {
        return path;
    }

    // Add approach
    addApproach(path, seam.points.front());

    // Add main path based on type
    switch (config_.type) {
        case PathType::SINGLE_PASS:
        case PathType::MULTI_PASS:
        case PathType::WEAVE:
            addWeldPath(path, seam);
            break;

        case PathType::TACK:
            addTackPath(path, seam);
            break;

        case PathType::STITCH:
        case PathType::SKIP:
            addStitchPath(path, seam);
            break;
    }

    // Add retract
    addRetract(path, seam.points.back());

    // Calculate statistics
    for (size_t i = 1; i < path.waypoints.size(); i++) {
        const auto& p0 = path.waypoints[i-1].pose;
        const auto& p1 = path.waypoints[i].pose;
        double dist = std::sqrt(
            std::pow(p1.x - p0.x, 2) +
            std::pow(p1.y - p0.y, 2) +
            std::pow(p1.z - p0.z, 2));

        path.total_length_mm += dist;

        if (path.waypoints[i].is_weld_on) {
            path.weld_length_mm += dist;
            path.estimated_weld_time_s += dist / path.waypoints[i].speed_mm_s;
        } else {
            path.travel_length_mm += dist;
        }

        path.estimated_total_time_s += dist / path.waypoints[i].speed_mm_s;
        path.estimated_total_time_s += path.waypoints[i].dwell_s;
    }

    return path;
}

void PathGenerator::addWeldPath(GeneratedPath& path, const DetectedSeam& seam) {
    for (int pass = 0; pass < config_.num_passes; pass++) {
        // Offset for multi-pass
        double offset = pass * config_.pass_offset_mm;

        for (const auto& sp : seam.points) {
            // Apply standoff and pass offset
            Eigen::Vector3d pos = sp.position +
                sp.normal * (config_.standoff_mm + offset);

            Waypoint wp;
            wp.pose = seamPointToPose(sp, config_.standoff_mm + offset);
            wp.speed_mm_s = config_.weld_speed_mm_s;
            wp.is_weld_on = true;
            wp.is_weave_on = config_.enable_weave;
            wp.type = Waypoint::Type::WELD;

            path.waypoints.push_back(wp);
        }

        // Return travel for multi-pass (reverse direction)
        if (pass < config_.num_passes - 1) {
            // Add retract-travel-approach sequence
            Waypoint retract;
            retract.pose = path.waypoints.back().pose;
            retract.pose.z += config_.retract_distance_mm;
            retract.speed_mm_s = config_.travel_speed_mm_s;
            retract.is_weld_on = false;
            retract.type = Waypoint::Type::RETRACT;
            path.waypoints.push_back(retract);

            Waypoint travel;
            travel.pose = seamPointToPose(seam.points.front(),
                config_.standoff_mm + (pass + 1) * config_.pass_offset_mm);
            travel.pose.z += config_.approach_distance_mm;
            travel.speed_mm_s = config_.travel_speed_mm_s;
            travel.is_weld_on = false;
            travel.type = Waypoint::Type::TRAVEL;
            path.waypoints.push_back(travel);
        }
    }
}

void PathGenerator::addTackPath(GeneratedPath& path, const DetectedSeam& seam) {
    // Distribute tacks evenly along seam
    int n = config_.num_tacks;
    if (n < 2) n = 2;

    double step = (seam.points.size() - 1) / (double)(n - 1);

    for (int i = 0; i < n; i++) {
        int idx = std::min((int)(i * step), (int)seam.points.size() - 1);
        const auto& sp = seam.points[idx];

        // Travel to tack position
        if (i > 0) {
            Waypoint travel;
            travel.pose = seamPointToPose(sp, config_.standoff_mm);
            travel.pose.z += config_.approach_distance_mm;
            travel.speed_mm_s = config_.travel_speed_mm_s;
            travel.is_weld_on = false;
            travel.type = Waypoint::Type::TRAVEL;
            path.waypoints.push_back(travel);
        }

        // Approach
        Waypoint approach;
        approach.pose = seamPointToPose(sp, config_.standoff_mm);
        approach.speed_mm_s = config_.approach_speed_mm_s;
        approach.is_weld_on = false;
        approach.type = Waypoint::Type::APPROACH;
        path.waypoints.push_back(approach);

        // Tack (dwell)
        Waypoint tack;
        tack.pose = seamPointToPose(sp, config_.standoff_mm);
        tack.speed_mm_s = 0;
        tack.is_weld_on = true;
        tack.dwell_s = config_.tack_dwell_s;
        tack.type = Waypoint::Type::WELD;
        path.waypoints.push_back(tack);

        // Retract
        Waypoint retract;
        retract.pose = seamPointToPose(sp, config_.standoff_mm);
        retract.pose.z += config_.retract_distance_mm;
        retract.speed_mm_s = config_.approach_speed_mm_s;
        retract.is_weld_on = false;
        retract.type = Waypoint::Type::RETRACT;
        path.waypoints.push_back(retract);
    }
}

Motion::CartesianPose PathGenerator::seamPointToPose(
    const SeamPoint& sp,
    double standoff_mm) {

    Motion::CartesianPose pose;

    // Position with standoff
    Eigen::Vector3d pos = sp.position + sp.approach * standoff_mm;
    pose.x = pos.x();
    pose.y = pos.y();
    pose.z = pos.z();

    // Orientation from approach and direction vectors
    Eigen::Vector3d z_axis = -sp.approach;  // Tool Z points opposite to approach
    Eigen::Vector3d x_axis = sp.direction.normalized();
    Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();
    x_axis = y_axis.cross(z_axis).normalized();

    Eigen::Matrix3d R;
    R.col(0) = x_axis;
    R.col(1) = y_axis;
    R.col(2) = z_axis;

    // Convert to Euler angles (ZYX convention)
    Eigen::Vector3d euler = R.eulerAngles(2, 1, 0) * 180.0 / M_PI;
    pose.rz = euler(0);
    pose.ry = euler(1);
    pose.rx = euler(2);

    return pose;
}

std::string PathGenerator::toGcode(const GeneratedPath& path) {
    std::ostringstream gcode;

    gcode << "; Generated path - " << path.description << "\n";
    gcode << "; Weld length: " << std::fixed << std::setprecision(1)
          << path.weld_length_mm << " mm\n";
    gcode << "; Estimated time: " << path.estimated_total_time_s << " s\n\n";

    gcode << "G90 ; Absolute positioning\n";
    gcode << "G21 ; Units: mm\n\n";

    bool weld_on = false;

    for (const auto& wp : path.waypoints) {
        // Weld on/off transitions
        if (wp.is_weld_on && !weld_on) {
            gcode << "M3 ; Weld ON\n";
            weld_on = true;
        } else if (!wp.is_weld_on && weld_on) {
            gcode << "M5 ; Weld OFF\n";
            weld_on = false;
        }

        // Movement
        gcode << "G1"
              << " X" << std::fixed << std::setprecision(3) << wp.pose.x
              << " Y" << wp.pose.y
              << " Z" << wp.pose.z
              << " A" << wp.pose.rx
              << " B" << wp.pose.ry
              << " C" << wp.pose.rz
              << " F" << std::setprecision(0) << wp.speed_mm_s * 60  // mm/min
              << "\n";

        // Dwell
        if (wp.dwell_s > 0) {
            gcode << "G4 P" << std::setprecision(2) << wp.dwell_s << "\n";
        }
    }

    if (weld_on) {
        gcode << "M5 ; Weld OFF\n";
    }

    gcode << "\nM30 ; Program end\n";

    return gcode.str();
}

} // namespace Scan
```

---

## 6. Scan-to-Path Workflow

### 6.1 FSM

```
                    ┌─────────┐
                    │  IDLE   │◄────────────────────────┐
                    └────┬────┘                         │
                         │ Start scan                   │
                         ▼                              │
                ┌────────────────┐                      │
                │  MOVE_TO_SCAN │                      │
                └───────┬────────┘                      │
                        │                               │
                        ▼                               │
                ┌────────────────┐                      │
                │   SCANNING    │ ◄──┐                 │
                │ (Capture PC)  │    │ More            │
                └───────┬────────┘    │ positions      │
                        │             │                 │
                        ├─────────────┘                 │
                        │ Scan complete                │
                        ▼                              │
                ┌────────────────┐                      │
                │  PROCESSING   │                      │
                │ (Filter/Merge)│                      │
                └───────┬────────┘                      │
                        │                               │
                        ▼                               │
                ┌────────────────┐                      │
                │ SEAM_DETECTION│── No seam found ─────┤
                └───────┬────────┘                      │
                        │ Seam found                   │
                        ▼                              │
                ┌────────────────┐                      │
                │ PATH_GENERATION│                     │
                └───────┬────────┘                      │
                        │                               │
                        ▼                               │
                ┌────────────────┐                      │
                │  PATH_PREVIEW │ ◄──┐                 │
                │ (User review) │    │ Edit            │
                └───────┬────────┘    │                 │
                        │             │                 │
                        ├─────────────┘                 │
                        │ Approve                      │
                        ▼                              │
                ┌────────────────┐                      │
                │  EXECUTING    │                      │
                │ (Run weld)    │                      │
                └───────┬────────┘                      │
                        │                               │
                        └───────────────────────────────┘
```

### 6.2 Scan Manager

```cpp
// ScanManager.h
#pragma once
#include "ScannerInterface.h"
#include "PointCloudProcessor.h"
#include "SeamDetector.h"
#include "PathGenerator.h"

namespace Scan {

enum class ScanState {
    IDLE,
    MOVE_TO_SCAN,
    SCANNING,
    PROCESSING,
    SEAM_DETECTION,
    PATH_GENERATION,
    PATH_PREVIEW,
    EXECUTING,
    ERROR,
    PAUSED
};

struct ScanConfig {
    // Scan positions (for multi-view scanning)
    std::vector<Motion::CartesianPose> scan_positions;

    // Single position mode
    Motion::CartesianPose single_scan_position;
    bool use_multi_view = false;

    // Processing
    ProcessingConfig processing;

    // Seam detection
    SeamDetectionConfig seam_detection;

    // Path generation
    PathConfig path;

    // Auto-execute after path generation
    bool auto_execute = false;
};

struct ScanResult {
    std::shared_ptr<PointCloud> merged_cloud;
    std::vector<DetectedSeam> seams;
    GeneratedPath path;

    bool success = false;
    std::string error_message;
};

class ScanManager {
public:
    ScanManager(
        std::unique_ptr<IScanner> scanner,
        Motion::TrajectoryPlanner& trajectory);

    // Configuration
    void configure(const ScanConfig& config);

    // Main workflow
    void start();
    void stop();
    void pause();
    void resume();

    // Manual steps
    bool captureCloud();
    bool processCloud();
    bool detectSeams();
    bool generatePath();
    void approvePath();
    void executePath();

    // Status
    ScanState getState() const { return state_; }
    const ScanResult& getResult() const { return result_; }
    float getProgress() const { return progress_; }

    // Cloud access (for visualization)
    std::shared_ptr<PointCloud> getCurrentCloud() const;
    const std::vector<DetectedSeam>& getDetectedSeams() const;
    const GeneratedPath& getGeneratedPath() const;

    // Callbacks
    using StateChangedCallback = std::function<void(ScanState)>;
    using CloudUpdatedCallback = std::function<void(std::shared_ptr<PointCloud>)>;
    using PathReadyCallback = std::function<void(const GeneratedPath&)>;

    void setStateChangedCallback(StateChangedCallback cb);
    void setCloudUpdatedCallback(CloudUpdatedCallback cb);
    void setPathReadyCallback(PathReadyCallback cb);

private:
    void stateMachine();
    void handleError(const std::string& message);

    std::unique_ptr<IScanner> scanner_;
    Motion::TrajectoryPlanner& trajectory_;
    PointCloudProcessor processor_;
    SeamDetector seam_detector_;
    PathGenerator path_generator_;

    ScanConfig config_;
    ScanState state_ = ScanState::IDLE;
    ScanResult result_;
    float progress_ = 0.0f;

    // Multi-view data
    std::vector<std::shared_ptr<PointCloud>> captured_clouds_;
    int current_scan_position_ = 0;

    // Callbacks
    StateChangedCallback state_changed_cb_;
    CloudUpdatedCallback cloud_updated_cb_;
    PathReadyCallback path_ready_cb_;
};

} // namespace Scan
```

---

## 7. CAD Comparison

### 7.1 Deviation Analysis

```cpp
// DeviationAnalyzer.h
#pragma once
#include "PointCloudProcessor.h"
#include <open3d/geometry/TriangleMesh.h>

namespace Scan {

struct DeviationResult {
    // Per-point deviations
    std::vector<double> distances_mm;

    // Statistics
    double min_deviation_mm;
    double max_deviation_mm;
    double mean_deviation_mm;
    double std_deviation_mm;
    double rms_deviation_mm;

    // Threshold analysis
    double in_tolerance_percent;  // % of points within tolerance

    // Color-coded cloud for visualization
    std::shared_ptr<PointCloud> colored_cloud;
};

struct DeviationConfig {
    double tolerance_mm = 0.5;

    // Color mapping
    double color_scale_min_mm = -1.0;
    double color_scale_max_mm = 1.0;

    // Alignment
    bool auto_align = true;
    double icp_max_distance_mm = 10.0;
};

class DeviationAnalyzer {
public:
    explicit DeviationAnalyzer(const DeviationConfig& config = {});

    // Compare scan to CAD mesh
    DeviationResult compare(
        const std::shared_ptr<PointCloud>& scan,
        const std::shared_ptr<open3d::geometry::TriangleMesh>& cad_mesh);

    // Compare scan to reference point cloud
    DeviationResult compare(
        const std::shared_ptr<PointCloud>& scan,
        const std::shared_ptr<PointCloud>& reference);

    // Load CAD from file
    std::shared_ptr<open3d::geometry::TriangleMesh> loadCAD(
        const std::string& filepath);  // STEP, STL, OBJ

    // Export report
    void exportReport(
        const DeviationResult& result,
        const std::string& filepath);

private:
    Eigen::Vector3f deviationToColor(double deviation_mm);

    DeviationConfig config_;
    PointCloudProcessor processor_;
};

} // namespace Scan
```

### 7.2 Implementation

```cpp
// DeviationAnalyzer.cpp
#include "DeviationAnalyzer.h"
#include <open3d/io/TriangleMeshIO.h>

namespace Scan {

DeviationResult DeviationAnalyzer::compare(
    const std::shared_ptr<PointCloud>& scan,
    const std::shared_ptr<open3d::geometry::TriangleMesh>& cad_mesh) {

    DeviationResult result;

    // Convert scan to Open3D
    auto o3d_scan = processor_.toOpen3D(scan);

    // Auto-align if enabled
    if (config_.auto_align) {
        // Sample points from mesh for ICP
        auto cad_cloud = cad_mesh->SamplePointsUniformly(100000);

        auto icp_result = open3d::pipelines::registration::RegistrationICP(
            *o3d_scan, *cad_cloud,
            config_.icp_max_distance_mm,
            Eigen::Matrix4d::Identity(),
            open3d::pipelines::registration::TransformationEstimationPointToPlane());

        o3d_scan->Transform(icp_result.transformation_);
    }

    // Compute distances to mesh
    auto scene = open3d::t::geometry::RaycastingScene();
    scene.AddTriangles(open3d::t::geometry::TriangleMesh::FromLegacy(*cad_mesh));

    auto points_tensor = open3d::core::Tensor(
        (void*)o3d_scan->points_.data(),
        {(int64_t)o3d_scan->points_.size(), 3},
        open3d::core::Dtype::Float64);

    auto distances_tensor = scene.ComputeDistance(points_tensor);
    auto signed_distances = scene.ComputeSignedDistance(points_tensor);

    // Extract results
    result.distances_mm.resize(scan->size());
    double sum = 0, sum_sq = 0;
    result.min_deviation_mm = DBL_MAX;
    result.max_deviation_mm = -DBL_MAX;
    int in_tolerance_count = 0;

    for (size_t i = 0; i < scan->size(); i++) {
        double d = signed_distances[i].Item<float>();
        result.distances_mm[i] = d;

        sum += d;
        sum_sq += d * d;
        result.min_deviation_mm = std::min(result.min_deviation_mm, d);
        result.max_deviation_mm = std::max(result.max_deviation_mm, d);

        if (std::abs(d) <= config_.tolerance_mm) {
            in_tolerance_count++;
        }
    }

    // Statistics
    size_t n = scan->size();
    result.mean_deviation_mm = sum / n;
    result.std_deviation_mm = std::sqrt(sum_sq / n -
        result.mean_deviation_mm * result.mean_deviation_mm);
    result.rms_deviation_mm = std::sqrt(sum_sq / n);
    result.in_tolerance_percent = 100.0 * in_tolerance_count / n;

    // Create colored cloud
    result.colored_cloud = std::make_shared<PointCloud>(*scan);
    for (size_t i = 0; i < result.colored_cloud->size(); i++) {
        auto color = deviationToColor(result.distances_mm[i]);
        result.colored_cloud->points[i].r = color(0) * 255;
        result.colored_cloud->points[i].g = color(1) * 255;
        result.colored_cloud->points[i].b = color(2) * 255;
    }

    return result;
}

Eigen::Vector3f DeviationAnalyzer::deviationToColor(double deviation_mm) {
    // Blue (-max) -> Green (0) -> Red (+max)
    double normalized = (deviation_mm - config_.color_scale_min_mm) /
        (config_.color_scale_max_mm - config_.color_scale_min_mm);
    normalized = std::clamp(normalized, 0.0, 1.0);

    Eigen::Vector3f color;
    if (normalized < 0.5) {
        // Blue to Green
        color(0) = 0.0f;
        color(1) = normalized * 2.0f;
        color(2) = 1.0f - normalized * 2.0f;
    } else {
        // Green to Red
        color(0) = (normalized - 0.5f) * 2.0f;
        color(1) = 1.0f - (normalized - 0.5f) * 2.0f;
        color(2) = 0.0f;
    }

    return color;
}

} // namespace Scan
```

---

## 8. HMI Integration

### 8.1 ScanView

```xml
<!-- ScanView.xaml -->
<UserControl x:Class="RobotHMI.Views.ScanView">
    <Grid>
        <Grid.ColumnDefinitions>
            <ColumnDefinition Width="3*"/>
            <ColumnDefinition Width="1*"/>
        </Grid.ColumnDefinitions>

        <!-- 3D Viewport -->
        <helix:HelixViewport3D Grid.Column="0" x:Name="Viewport"
                               Background="#1E1E1E"
                               ShowCoordinateSystem="True">

            <!-- Point Cloud Display -->
            <helix:PointsVisual3D Points="{Binding PointCloudPoints}"
                                   Color="{Binding PointCloudColor}"
                                   Size="2"/>

            <!-- Detected Seams -->
            <helix:LinesVisual3D Points="{Binding SeamLinePoints}"
                                  Color="Yellow"
                                  Thickness="3"/>

            <!-- Generated Path -->
            <helix:TubeVisual3D Path="{Binding PathPoints}"
                                Diameter="2"
                                Material="{Binding PathMaterial}"/>

            <!-- Robot Model -->
            <helix:ModelVisual3D Content="{Binding RobotModel}"/>

        </helix:HelixViewport3D>

        <!-- Control Panel -->
        <ScrollViewer Grid.Column="1">
            <StackPanel Margin="10">

                <!-- State Display -->
                <GroupBox Header="Status">
                    <StackPanel>
                        <TextBlock Text="{Binding CurrentState}"
                                   FontSize="16" FontWeight="Bold"/>
                        <ProgressBar Value="{Binding Progress}"
                                     Height="20" Margin="0,10"/>
                    </StackPanel>
                </GroupBox>

                <!-- Scanner Control -->
                <GroupBox Header="Scanner">
                    <StackPanel>
                        <ComboBox ItemsSource="{Binding ScannerTypes}"
                                  SelectedItem="{Binding SelectedScanner}"/>
                        <StackPanel Orientation="Horizontal" Margin="0,10">
                            <Button Content="Connect"
                                    Command="{Binding ConnectScannerCommand}"
                                    Width="80" Margin="0,0,5,0"/>
                            <Ellipse Width="12" Height="12"
                                     Fill="{Binding ScannerStatusColor}"/>
                        </StackPanel>
                    </StackPanel>
                </GroupBox>

                <!-- Workflow Controls -->
                <GroupBox Header="Workflow">
                    <StackPanel>
                        <Button Content="SCAN"
                                Command="{Binding StartScanCommand}"
                                Height="40" Margin="0,5"/>
                        <Button Content="DETECT SEAM"
                                Command="{Binding DetectSeamCommand}"
                                Height="40" Margin="0,5"/>
                        <Button Content="GENERATE PATH"
                                Command="{Binding GeneratePathCommand}"
                                Height="40" Margin="0,5"/>
                        <Button Content="EXECUTE"
                                Command="{Binding ExecuteCommand}"
                                Height="40" Margin="0,5"
                                Background="#4CAF50"/>
                    </StackPanel>
                </GroupBox>

                <!-- Seam Info -->
                <GroupBox Header="Detected Seam"
                          Visibility="{Binding HasSeam, Converter={StaticResource BoolToVis}}">
                    <StackPanel>
                        <TextBlock Text="{Binding SeamType}"/>
                        <TextBlock Text="{Binding SeamLength, StringFormat='Length: {0:F1} mm'}"/>
                        <TextBlock Text="{Binding SeamConfidence, StringFormat='Confidence: {0:P0}'}"/>
                    </StackPanel>
                </GroupBox>

                <!-- Path Info -->
                <GroupBox Header="Generated Path"
                          Visibility="{Binding HasPath, Converter={StaticResource BoolToVis}}">
                    <StackPanel>
                        <TextBlock Text="{Binding PathWeldLength, StringFormat='Weld: {0:F1} mm'}"/>
                        <TextBlock Text="{Binding PathTravelLength, StringFormat='Travel: {0:F1} mm'}"/>
                        <TextBlock Text="{Binding PathEstimatedTime, StringFormat='Est. Time: {0:F0} s'}"/>
                        <Button Content="Preview Animation"
                                Command="{Binding PreviewPathCommand}"
                                Margin="0,10"/>
                    </StackPanel>
                </GroupBox>

                <!-- Point Cloud Stats -->
                <GroupBox Header="Point Cloud">
                    <StackPanel>
                        <TextBlock Text="{Binding PointCount, StringFormat='Points: {0:N0}'}"/>
                        <TextBlock Text="{Binding CloudBounds}"/>
                    </StackPanel>
                </GroupBox>

            </StackPanel>
        </ScrollViewer>
    </Grid>
</UserControl>
```

### 8.2 ViewModel

```csharp
// ScanViewModel.cs
public partial class ScanViewModel : ObservableObject
{
    private readonly IScanService _scanService;
    private readonly IPointCloudRenderer _renderer;

    [ObservableProperty]
    private string _currentState = "IDLE";

    [ObservableProperty]
    private double _progress;

    [ObservableProperty]
    private Point3DCollection _pointCloudPoints = new();

    [ObservableProperty]
    private Point3DCollection _seamLinePoints = new();

    [ObservableProperty]
    private Point3DCollection _pathPoints = new();

    [ObservableProperty]
    private int _pointCount;

    [ObservableProperty]
    private string _seamType = "";

    [ObservableProperty]
    private double _seamLength;

    [ObservableProperty]
    private double _seamConfidence;

    [ObservableProperty]
    private bool _hasSeam;

    [ObservableProperty]
    private bool _hasPath;

    [RelayCommand]
    private async Task StartScan()
    {
        CurrentState = "SCANNING";
        var cloud = await _scanService.CaptureAsync();

        if (cloud != null)
        {
            UpdatePointCloud(cloud);
            CurrentState = "SCAN_COMPLETE";
        }
    }

    [RelayCommand]
    private async Task DetectSeam()
    {
        CurrentState = "DETECTING";
        var seams = await _scanService.DetectSeamsAsync();

        if (seams.Any())
        {
            var primarySeam = seams.First();
            SeamType = primarySeam.Type.ToString();
            SeamLength = primarySeam.TotalLengthMm;
            SeamConfidence = primarySeam.Confidence;
            HasSeam = true;

            UpdateSeamVisualization(primarySeam);
            CurrentState = "SEAM_DETECTED";
        }
        else
        {
            CurrentState = "NO_SEAM_FOUND";
        }
    }

    [RelayCommand]
    private async Task GeneratePath()
    {
        CurrentState = "GENERATING_PATH";
        var path = await _scanService.GeneratePathAsync();

        if (path.Waypoints.Any())
        {
            HasPath = true;
            UpdatePathVisualization(path);
            CurrentState = "PATH_READY";
        }
    }

    [RelayCommand]
    private async Task Execute()
    {
        CurrentState = "EXECUTING";
        await _scanService.ExecutePathAsync();
        CurrentState = "COMPLETE";
    }

    private void UpdatePointCloud(PointCloudDto cloud)
    {
        var points = new Point3DCollection();
        foreach (var pt in cloud.Points)
        {
            points.Add(new Point3D(pt.X, pt.Y, pt.Z));
        }

        PointCloudPoints = points;
        PointCount = cloud.Points.Length;
    }

    private void UpdateSeamVisualization(SeamDto seam)
    {
        var points = new Point3DCollection();
        foreach (var pt in seam.Points)
        {
            points.Add(new Point3D(pt.X, pt.Y, pt.Z));
        }
        SeamLinePoints = points;
    }

    private void UpdatePathVisualization(PathDto path)
    {
        var points = new Point3DCollection();
        foreach (var wp in path.Waypoints)
        {
            points.Add(new Point3D(wp.X, wp.Y, wp.Z));
        }
        PathPoints = points;
    }
}
```

---

## 9. Configuration

### 9.1 Scan Config File

```yaml
# config/scan_to_path.yaml
scanner:
  type: REALSENSE_D435  # REALSENSE_D435, REALSENSE_L515, ZIVID_ONE_PLUS
  device_id: ""  # Auto-detect if empty

  resolution:
    width: 1280
    height: 720

  depth:
    min_m: 0.3
    max_m: 1.2

  filters:
    decimation: true
    decimation_factor: 2
    spatial: true
    temporal: true

  exposure_us: 8500
  laser_power: 150

processing:
  voxel_size_mm: 1.0

  outlier_removal:
    neighbors: 20
    std_ratio: 2.0

  normal_estimation:
    neighbors: 30

seam_detection:
  edge_angle_threshold_deg: 30.0
  min_edge_length_mm: 20.0
  max_gap_width_mm: 10.0
  min_gap_width_mm: 0.5
  sample_spacing_mm: 2.0

path_generation:
  type: SINGLE_PASS  # SINGLE_PASS, MULTI_PASS, WEAVE, TACK, STITCH

  speeds:
    travel_mm_s: 500.0
    weld_mm_s: 10.0
    approach_mm_s: 100.0

  distances:
    approach_mm: 30.0
    retract_mm: 30.0
    standoff_mm: 15.0

  multi_pass:
    num_passes: 1
    pass_offset_mm: 3.0

  weaving:
    enable: false
    width_mm: 5.0
    frequency_hz: 2.0

scan_positions:
  use_multi_view: false
  positions:
    - { x: 300, y: 0, z: 400, rx: 180, ry: 0, rz: 0 }
    # Add more for multi-view scanning
```

### 9.2 Scanner Calibration File

```yaml
# config/scanner_calibration.yaml
scanner_extrinsics:
  # T_flange_to_scanner (4x4 matrix, row-major)
  matrix:
    - [1.0, 0.0, 0.0, 50.0]   # X offset from flange
    - [0.0, 1.0, 0.0, 0.0]    # Y offset
    - [0.0, 0.0, 1.0, 80.0]   # Z offset
    - [0.0, 0.0, 0.0, 1.0]

  calibration_date: "2026-01-20"
  method: "hand_eye_tsai"

camera_intrinsics:
  # RealSense D435
  fx: 615.2
  fy: 615.4
  cx: 323.5
  cy: 238.8
  width: 640
  height: 480

  distortion:
    model: "brown_conrady"
    coeffs: [0.0, 0.0, 0.0, 0.0, 0.0]
```

---

## 10. Testing

### 10.1 Unit Tests

```cpp
// test_scan_to_path.cpp
#include <gtest/gtest.h>
#include "SeamDetector.h"
#include "PathGenerator.h"
#include "DeviationAnalyzer.h"

class SeamDetectorTest : public ::testing::Test {
protected:
    void SetUp() override {
        detector_ = std::make_unique<SeamDetector>();
    }

    std::unique_ptr<SeamDetector> detector_;
};

TEST_F(SeamDetectorTest, DetectsFilletJoint) {
    // Create synthetic point cloud with fillet joint
    auto cloud = createFilletJointCloud(
        /*angle=*/90.0,
        /*length=*/100.0,
        /*noise=*/0.5);

    auto seams = detector_->detectSeams(cloud);

    ASSERT_GE(seams.size(), 1);
    EXPECT_EQ(seams[0].type, SeamType::FILLET);
    EXPECT_NEAR(seams[0].total_length_mm, 100.0, 5.0);
    EXPECT_GT(seams[0].confidence, 0.8);
}

TEST_F(SeamDetectorTest, DetectsButtJoint) {
    auto cloud = createButtJointCloud(
        /*gap_mm=*/2.0,
        /*length=*/150.0);

    auto seams = detector_->detectSeams(cloud);

    ASSERT_GE(seams.size(), 1);
    EXPECT_EQ(seams[0].type, SeamType::BUTT);

    // Check gap detection
    for (const auto& pt : seams[0].points) {
        EXPECT_NEAR(pt.gap_width_mm, 2.0, 0.5);
    }
}

class PathGeneratorTest : public ::testing::Test {
protected:
    void SetUp() override {
        PathConfig config;
        config.standoff_mm = 15.0;
        config.weld_speed_mm_s = 10.0;
        generator_ = std::make_unique<PathGenerator>(config);
    }

    std::unique_ptr<PathGenerator> generator_;
};

TEST_F(PathGeneratorTest, GeneratesSinglePass) {
    DetectedSeam seam;
    seam.type = SeamType::FILLET;

    // Add seam points
    for (int i = 0; i < 100; i++) {
        SeamPoint sp;
        sp.position = Eigen::Vector3d(i, 0, 0);
        sp.direction = Eigen::Vector3d(1, 0, 0);
        sp.normal = Eigen::Vector3d(0, 0.707, 0.707);
        sp.approach = Eigen::Vector3d(0, -0.707, -0.707);
        seam.points.push_back(sp);
    }
    seam.total_length_mm = 99.0;

    auto path = generator_->generateFromSeam(seam);

    EXPECT_GT(path.waypoints.size(), 100);  // Including approach/retract
    EXPECT_NEAR(path.weld_length_mm, 99.0, 1.0);

    // Check welding is on for main path
    int weld_on_count = 0;
    for (const auto& wp : path.waypoints) {
        if (wp.is_weld_on) weld_on_count++;
    }
    EXPECT_EQ(weld_on_count, 100);
}

TEST_F(PathGeneratorTest, GeneratesTackPattern) {
    PathConfig config;
    config.type = PathType::TACK;
    config.num_tacks = 5;
    config.tack_dwell_s = 1.0;
    generator_ = std::make_unique<PathGenerator>(config);

    DetectedSeam seam = createLinearSeam(100.0);
    auto path = generator_->generateFromSeam(seam);

    // Count dwell points
    int dwell_count = 0;
    for (const auto& wp : path.waypoints) {
        if (wp.dwell_s > 0) dwell_count++;
    }
    EXPECT_EQ(dwell_count, 5);
}

class DeviationAnalyzerTest : public ::testing::Test {
protected:
    void SetUp() override {
        DeviationConfig config;
        config.tolerance_mm = 0.5;
        analyzer_ = std::make_unique<DeviationAnalyzer>(config);
    }

    std::unique_ptr<DeviationAnalyzer> analyzer_;
};

TEST_F(DeviationAnalyzerTest, PerfectMatchZeroDeviation) {
    auto mesh = createFlatPlaneMesh(100, 100);
    auto scan = sampleMeshToCloud(mesh, 10000);

    auto result = analyzer_->compare(scan, mesh);

    EXPECT_NEAR(result.mean_deviation_mm, 0.0, 0.01);
    EXPECT_NEAR(result.rms_deviation_mm, 0.0, 0.01);
    EXPECT_EQ(result.in_tolerance_percent, 100.0);
}

TEST_F(DeviationAnalyzerTest, DetectsKnownDeviation) {
    auto mesh = createFlatPlaneMesh(100, 100);
    auto scan = sampleMeshToCloud(mesh, 10000);

    // Add known offset
    for (auto& pt : scan->points) {
        pt.z += 0.3;  // 0.3mm offset
    }

    auto result = analyzer_->compare(scan, mesh);

    EXPECT_NEAR(result.mean_deviation_mm, 0.3, 0.05);
    EXPECT_GT(result.in_tolerance_percent, 90.0);  // Most still in tolerance
}
```

### 10.2 Integration Test Checklist

```
Scan-to-Path Integration Tests:

[ ] Scanner
    [ ] Connection to RealSense/Zivid
    [ ] Frame capture
    [ ] Point cloud generation
    [ ] Multi-view capture

[ ] Processing
    [ ] Voxel downsampling
    [ ] Outlier removal
    [ ] Normal estimation
    [ ] ICP registration

[ ] Seam Detection
    [ ] Fillet joint detection
    [ ] Butt joint detection
    [ ] Lap joint detection
    [ ] Gap measurement
    [ ] Seam curve fitting

[ ] Path Generation
    [ ] Single pass path
    [ ] Multi-pass path
    [ ] Tack pattern
    [ ] Stitch pattern
    [ ] G-code export

[ ] Workflow
    [ ] Full scan-to-path cycle
    [ ] Path preview
    [ ] Path editing
    [ ] Execution

[ ] CAD Comparison
    [ ] STEP file loading
    [ ] Deviation calculation
    [ ] Color-coded visualization
    [ ] Report generation
```

---

## 11. Document History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 0.1 | 2026-02-01 | System | Initial draft |

---

## 12. References

- [Intel RealSense SDK 2.0] librealsense documentation
- [Open3D] Point cloud processing library
- [PCL] Point Cloud Library
- CORE_02_Kinematics.md - Robot coordinate transforms
- CORE_03_Trajectory.md - Motion planning
- MODE_01_Welding.md - Welding execution
