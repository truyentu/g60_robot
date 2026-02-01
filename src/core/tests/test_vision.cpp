#include <gtest/gtest.h>
#include "vision/VisionTypes.hpp"

using namespace robot_controller::vision;

TEST(Point3DTest, DefaultConstruction) {
    Point3D p;
    EXPECT_FLOAT_EQ(p.x, 0.0f);
    EXPECT_FLOAT_EQ(p.y, 0.0f);
    EXPECT_FLOAT_EQ(p.z, 0.0f);
    EXPECT_FLOAT_EQ(p.intensity, 0.0f);
}

TEST(Point3DTest, ParameterizedConstruction) {
    Point3D p(1.0f, 2.0f, 3.0f, 100.0f);
    EXPECT_FLOAT_EQ(p.x, 1.0f);
    EXPECT_FLOAT_EQ(p.y, 2.0f);
    EXPECT_FLOAT_EQ(p.z, 3.0f);
    EXPECT_FLOAT_EQ(p.intensity, 100.0f);
}

TEST(Point2DTest, DefaultConstruction) {
    Point2D p;
    EXPECT_FLOAT_EQ(p.x, 0.0f);
    EXPECT_FLOAT_EQ(p.z, 0.0f);
    EXPECT_FLOAT_EQ(p.intensity, 0.0f);
    EXPECT_TRUE(p.valid);
}

TEST(Point2DTest, ParameterizedConstruction) {
    Point2D p(5.0f, 10.0f, 128.0f, false);
    EXPECT_FLOAT_EQ(p.x, 5.0f);
    EXPECT_FLOAT_EQ(p.z, 10.0f);
    EXPECT_FLOAT_EQ(p.intensity, 128.0f);
    EXPECT_FALSE(p.valid);
}

TEST(LaserProfileTest, ComputeStats) {
    LaserProfile profile;
    profile.points = {
        Point2D(0, 10, 100, true),
        Point2D(1, 20, 100, true),
        Point2D(2, 15, 100, false),  // Invalid
        Point2D(3, 30, 100, true)
    };

    profile.computeStats();

    EXPECT_EQ(profile.validPointCount, 3u);
    EXPECT_FLOAT_EQ(profile.minZ, 10.0f);
    EXPECT_FLOAT_EQ(profile.maxZ, 30.0f);
    EXPECT_FLOAT_EQ(profile.avgIntensity, 100.0f);
}

TEST(LaserProfileTest, EmptyProfile) {
    LaserProfile profile;
    profile.computeStats();

    EXPECT_EQ(profile.validPointCount, 0u);
}

TEST(PointCloudTest, ComputeBounds) {
    PointCloud cloud;
    cloud.points = {
        Point3D(0, 0, 0),
        Point3D(10, 20, 30),
        Point3D(-5, -10, 5)
    };

    cloud.computeBounds();

    EXPECT_FLOAT_EQ(cloud.minBound.x, -5.0f);
    EXPECT_FLOAT_EQ(cloud.minBound.y, -10.0f);
    EXPECT_FLOAT_EQ(cloud.minBound.z, 0.0f);
    EXPECT_FLOAT_EQ(cloud.maxBound.x, 10.0f);
    EXPECT_FLOAT_EQ(cloud.maxBound.y, 20.0f);
    EXPECT_FLOAT_EQ(cloud.maxBound.z, 30.0f);
}

TEST(PointCloudTest, SizeAndEmpty) {
    PointCloud cloud;
    EXPECT_TRUE(cloud.empty());
    EXPECT_EQ(cloud.size(), 0u);

    cloud.points.push_back(Point3D(0, 0, 0));
    EXPECT_FALSE(cloud.empty());
    EXPECT_EQ(cloud.size(), 1u);
}

TEST(PointCloudTest, IsOrganized) {
    PointCloud cloud;
    EXPECT_FALSE(cloud.isOrganized());

    cloud.width = 640;
    cloud.height = 480;
    EXPECT_TRUE(cloud.isOrganized());
}

TEST(LaserProfilerConfigTest, DefaultValues) {
    LaserProfilerConfig config;

    EXPECT_EQ(config.profileWidth, 2048u);
    EXPECT_FLOAT_EQ(config.exposureTime, 1000.0f);
    EXPECT_EQ(config.triggerMode, LaserProfilerConfig::TriggerMode::FreeRun);
    EXPECT_TRUE(config.enableIntensityFilter);
    EXPECT_FALSE(config.enableMedianFilter);
}

TEST(HandEyeCalibrationTest, DefaultInvalid) {
    HandEyeCalibration cal;
    EXPECT_FALSE(cal.valid);
}

TEST(ProfileBatchTest, TotalPoints) {
    ProfileBatch batch;

    LaserProfile p1;
    p1.validPointCount = 100;

    LaserProfile p2;
    p2.validPointCount = 200;

    batch.profiles.push_back(p1);
    batch.profiles.push_back(p2);

    EXPECT_EQ(batch.totalPoints(), 300u);
}

TEST(ProfileBatchTest, EmptyBatch) {
    ProfileBatch batch;
    EXPECT_EQ(batch.totalPoints(), 0u);
}

TEST(RobotPoseTest, DefaultInvalid) {
    RobotPose pose;
    EXPECT_FALSE(pose.valid);
    EXPECT_EQ(pose.timestamp, 0u);
}

TEST(CameraFrameTest, DataSize) {
    CameraFrame frame;
    frame.width = 640;
    frame.height = 480;
    frame.bytesPerPixel = 1;

    EXPECT_EQ(frame.dataSize(), 640u * 480u);

    frame.bytesPerPixel = 3;
    EXPECT_EQ(frame.dataSize(), 640u * 480u * 3u);
}

TEST(SensorInfoTest, DefaultStatus) {
    SensorInfo info;
    EXPECT_EQ(info.status, SensorStatus::Disconnected);
}

TEST(DepthFrameTest, DepthScale) {
    DepthFrame frame;
    EXPECT_FLOAT_EQ(frame.depthScale, 0.001f);
}
