cmake_minimum_required(VERSION 3.20)
project(RobotControllerCore VERSION 1.0.0 LANGUAGES CXX)

# ============================================================================
# Build Settings
# ============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Export compile commands for IDE integration
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# ============================================================================
# vcpkg Integration
# ============================================================================
if(DEFINED ENV{VCPKG_ROOT})
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
        CACHE STRING "Vcpkg toolchain file")
elseif(EXISTS "C:/vcpkg/scripts/buildsystems/vcpkg.cmake")
    set(CMAKE_TOOLCHAIN_FILE "C:/vcpkg/scripts/buildsystems/vcpkg.cmake"
        CACHE STRING "Vcpkg toolchain file")
endif()

# ============================================================================
# Find Dependencies
# ============================================================================
find_package(spdlog CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(cppzmq CONFIG REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)
find_package(GTest CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(ruckig CONFIG REQUIRED)
find_package(orocos_kdl CONFIG REQUIRED)

# ============================================================================
# Source Files
# ============================================================================
set(CORE_SOURCES
    src/main.cpp
    src/ipc/IpcServer.cpp
    src/ipc/RobotPackagePayloads.cpp
    src/ipc/JogPayloads.cpp
    src/firmware/FirmwareSimulator.cpp
    src/firmware/STM32EthernetDriver.cpp
    src/firmware/plugins/GcodeAdapter.cpp
    src/jog/JogController.cpp
    src/jog/RuckigSmoother.cpp
    src/config/ConfigManager.cpp
    src/config/RobotCatalog.cpp
    src/config/RobotInstance.cpp
    src/config/RobotPackageLoader.cpp
    src/config/UrdfParser.cpp
    src/robot/RobotModel.cpp
    src/logging/Logger.cpp
    src/state/StateMachine.cpp
    src/controller/RobotController.cpp
    src/homing/HomingService.cpp
    src/tool/ToolTypes.cpp
    src/tool/ToolManager.cpp
    src/tool/ToolCalibration.cpp
    src/mode/ModeManager.cpp
    src/frame/BaseFrameManager.cpp
    src/frame/BaseCalibration.cpp
    src/interpreter/Lexer.cpp
    src/interpreter/Parser.cpp
    src/interpreter/Executor.cpp
    src/kinematics/UrdfForwardKinematics.cpp
    src/kinematics/KDLKinematics.cpp
    src/kinematics/AnalyticalKinematics.cpp
)

set(CORE_HEADERS
    include/robot_controller/core.hpp
    src/ipc/IpcServer.hpp
    src/ipc/Message.hpp
    src/ipc/MessageTypes.hpp
    src/config/ConfigManager.hpp
    src/config/RobotConfig.hpp
    src/config/SystemConfig.hpp
    src/config/RobotCatalog.hpp
    src/config/RobotInstance.hpp
    src/config/RobotPackageSchema.hpp
    src/config/RobotPackageLoader.hpp
    src/robot/RobotModel.hpp
    src/robot/DHParameters.hpp
    src/logging/Logger.hpp
    src/state/States.hpp
    src/state/IStateMachine.hpp
    src/state/StateMachine.hpp
    src/controller/RobotController.hpp
    src/kinematics/MathTypes.hpp
    src/kinematics/DHParameters.hpp
    src/kinematics/ForwardKinematics.hpp
    src/kinematics/InverseKinematics.hpp
    src/kinematics/KinematicsService.hpp
    src/kinematics/UrdfForwardKinematics.hpp
    src/kinematics/KDLKinematics.hpp
    src/kinematics/AnalyticalKinematics.hpp
    src/trajectory/TrajectoryTypes.hpp
    src/trajectory/VelocityProfile.hpp
    src/trajectory/Interpolators.hpp
    src/trajectory/TrajectoryPlanner.hpp
    src/trajectory/TrajectoryExecutor.hpp
    src/firmware/LegacyGrblProtocol.hpp
    src/firmware/protocol/PacketTypes.hpp
    src/firmware/protocol/PVTPoint.hpp
    src/firmware/protocol/StatusPacket.hpp
    src/firmware/protocol/BinaryProtocol.hpp
    src/firmware/STM32EthernetDriver.hpp
    src/firmware/plugins/GcodeAdapter.hpp
    src/welding/WeldingTypes.hpp
    src/welding/WeldingStateMachine.hpp
    src/welding/WeldingController.hpp
    src/welding/WeldingIO.hpp
    src/ipc/WeldingPayloads.hpp
    src/ipc/WeavePayloads.hpp
    src/welding/WeavingTypes.hpp
    src/welding/WeavePatternGenerator.hpp
    src/welding/WeaveExecutor.hpp
    src/welding/WeaveVisualizer.hpp
    src/vision/VisionTypes.hpp
    src/vision/ILaserProfiler.hpp
    src/vision/HikrobotLaserProfiler.hpp
    src/vision/SensorManager.hpp
    src/ipc/SensorPayloads.hpp
    src/vision/SeamTypes.hpp
    src/vision/ProfileProcessor.hpp
    src/vision/JointDetector.hpp
    src/vision/SeamTracker.hpp
    src/ipc/SeamPayloads.hpp
    src/homing/HomingTypes.hpp
    src/homing/HomingService.hpp
    src/ipc/HomingPayloads.hpp
    src/tool/ToolTypes.hpp
    src/tool/ToolManager.hpp
    src/tool/ToolCalibration.hpp
    src/ipc/ToolPayloads.hpp
    src/mode/OperationMode.hpp
    src/mode/ModeManager.hpp
    src/ipc/ModePayloads.hpp
    src/frame/FrameTypes.hpp
    src/frame/BaseFrameManager.hpp
    src/frame/BaseCalibration.hpp
    src/ipc/BasePayloads.hpp
    src/ipc/RobotPackagePayloads.hpp
    src/ipc/JogPayloads.hpp
    src/firmware/FirmwareSimulator.hpp
    src/firmware/IFirmwareDriver.hpp
    src/jog/JogController.hpp
    src/jog/RuckigSmoother.hpp
    src/ipc/OverridePayloads.hpp
    src/interpreter/Token.hpp
    src/interpreter/Lexer.hpp
    src/interpreter/AST.hpp
    src/interpreter/Parser.hpp
    src/interpreter/Executor.hpp
)

# ============================================================================
# Main Executable
# ============================================================================
add_executable(robot_core ${CORE_SOURCES} ${CORE_HEADERS})

target_include_directories(robot_core
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(robot_core
    PRIVATE
        spdlog::spdlog
        nlohmann_json::nlohmann_json
        cppzmq
        yaml-cpp::yaml-cpp
        Eigen3::Eigen
        Boost::system
        ruckig::ruckig
        orocos-kdl
)

# Windows-specific settings
if(WIN32)
    target_compile_definitions(robot_core PRIVATE
        _WIN32_WINNT=0x0A00  # Windows 10
        NOMINMAX             # Prevent min/max macro conflicts
    )
endif()

# ============================================================================
# Tests
# ============================================================================
enable_testing()

add_executable(robot_core_tests
    tests/test_main.cpp
    tests/test_config.cpp
    tests/test_ipc.cpp
    tests/test_state_machine.cpp
    src/config/ConfigManager.cpp
    src/config/RobotCatalog.cpp
    src/config/RobotInstance.cpp
    src/logging/Logger.cpp
    src/ipc/IpcServer.cpp
    src/state/StateMachine.cpp
)

target_include_directories(robot_core_tests
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(robot_core_tests
    PRIVATE
        GTest::gtest
        GTest::gtest_main
        spdlog::spdlog
        nlohmann_json::nlohmann_json
        cppzmq
        yaml-cpp::yaml-cpp
)

include(GoogleTest)
gtest_discover_tests(robot_core_tests)

# ============================================================================
# Kinematics Tests
# ============================================================================
add_executable(test_kinematics
    tests/test_kinematics.cpp
    src/kinematics/UrdfForwardKinematics.cpp
    src/kinematics/KDLKinematics.cpp
    src/logging/Logger.cpp
)

target_include_directories(test_kinematics
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(test_kinematics
    PRIVATE
        GTest::gtest
        GTest::gtest_main
        Eigen3::Eigen
        orocos-kdl
        spdlog::spdlog
)

if(WIN32)
    target_compile_definitions(test_kinematics PRIVATE
        NOMINMAX
    )
    target_compile_options(test_kinematics PRIVATE /utf-8)
endif()

gtest_discover_tests(test_kinematics)

# ============================================================================
# URDF FK Tests
# ============================================================================
add_executable(test_urdf_fk
    tests/test_urdf_fk.cpp
    src/kinematics/UrdfForwardKinematics.cpp
    src/logging/Logger.cpp
)

target_include_directories(test_urdf_fk
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(test_urdf_fk
    PRIVATE
        GTest::gtest
        GTest::gtest_main
        Eigen3::Eigen
        spdlog::spdlog
)

if(WIN32)
    target_compile_definitions(test_urdf_fk PRIVATE
        NOMINMAX
    )
endif()

gtest_discover_tests(test_urdf_fk)

# ============================================================================
# KDL Kinematics Tests (FK cross-validation + IK round-trip)
# ============================================================================
add_executable(test_kdl_kinematics
    tests/test_kdl_kinematics.cpp
    src/kinematics/UrdfForwardKinematics.cpp
    src/kinematics/KDLKinematics.cpp
    src/logging/Logger.cpp
)

target_include_directories(test_kdl_kinematics
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(test_kdl_kinematics
    PRIVATE
        GTest::gtest
        GTest::gtest_main
        Eigen3::Eigen
        spdlog::spdlog
        orocos-kdl
)

if(WIN32)
    target_compile_definitions(test_kdl_kinematics PRIVATE
        NOMINMAX
    )
endif()

gtest_discover_tests(test_kdl_kinematics)

# ============================================================================
# Analytical Kinematics Tests (DH FK/IK round-trip)
# ============================================================================
add_executable(test_analytical_kinematics
    tests/test_analytical_kinematics.cpp
    src/kinematics/AnalyticalKinematics.cpp
)

target_include_directories(test_analytical_kinematics
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(test_analytical_kinematics
    PRIVATE
        GTest::gtest
        GTest::gtest_main
        Eigen3::Eigen
)

if(WIN32)
    target_compile_definitions(test_analytical_kinematics PRIVATE
        NOMINMAX
    )
    target_compile_options(test_analytical_kinematics PRIVATE /utf-8)
endif()

gtest_discover_tests(test_analytical_kinematics)

# ============================================================================
# Double S-Curve Velocity Profile Tests
# ============================================================================
add_executable(test_double_scurve
    tests/test_double_scurve.cpp
    src/trajectory/DoubleSCurveProfile.cpp
)

target_include_directories(test_double_scurve
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(test_double_scurve
    PRIVATE
        GTest::gtest
        GTest::gtest_main
)

if(WIN32)
    target_compile_definitions(test_double_scurve PRIVATE
        NOMINMAX
    )
endif()

gtest_discover_tests(test_double_scurve)

# ============================================================================
# Cartesian Interpolator Tests (MoveL & MoveC)
# ============================================================================
add_executable(test_cartesian_interpolator
    tests/test_cartesian_interpolator.cpp
    src/trajectory/CartesianInterpolator.cpp
    src/trajectory/DoubleSCurveProfile.cpp
)

target_include_directories(test_cartesian_interpolator
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(test_cartesian_interpolator
    PRIVATE
        GTest::gtest
        GTest::gtest_main
        Eigen3::Eigen
)

if(WIN32)
    target_compile_definitions(test_cartesian_interpolator PRIVATE
        NOMINMAX
        _USE_MATH_DEFINES
    )
    target_compile_options(test_cartesian_interpolator PRIVATE /utf-8)
endif()

gtest_discover_tests(test_cartesian_interpolator)

# ============================================================================
# Look-Ahead Optimizer Tests
# ============================================================================
add_executable(test_look_ahead
    tests/test_look_ahead.cpp
    src/trajectory/LookAheadOptimizer.cpp
)

target_include_directories(test_look_ahead
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(test_look_ahead
    PRIVATE
        GTest::gtest
        GTest::gtest_main
)

if(WIN32)
    target_compile_definitions(test_look_ahead PRIVATE
        NOMINMAX
        _USE_MATH_DEFINES
    )
    target_compile_options(test_look_ahead PRIVATE /utf-8)
endif()

gtest_discover_tests(test_look_ahead)

# ============================================================================
# MA2010 Critical Geometry Tests (h3/h4 sign & theta5 negation verification)
# ============================================================================
add_executable(test_ma2010_critical_geometry
    tests/test_ma2010_critical_geometry.cpp
    src/kinematics/AnalyticalKinematics.cpp
)

target_include_directories(test_ma2010_critical_geometry
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(test_ma2010_critical_geometry
    PRIVATE
        GTest::gtest
        GTest::gtest_main
        Eigen3::Eigen
)

if(WIN32)
    target_compile_definitions(test_ma2010_critical_geometry PRIVATE
        NOMINMAX
    )
    target_compile_options(test_ma2010_critical_geometry PRIVATE /utf-8)
endif()

gtest_discover_tests(test_ma2010_critical_geometry)

# ============================================================================
# Trajectory Tests
# ============================================================================
add_executable(test_trajectory
    tests/test_trajectory.cpp
    src/kinematics/UrdfForwardKinematics.cpp
    src/kinematics/KDLKinematics.cpp
    src/logging/Logger.cpp
)

target_include_directories(test_trajectory
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(test_trajectory
    PRIVATE
        GTest::gtest
        GTest::gtest_main
        Eigen3::Eigen
        orocos-kdl
        spdlog::spdlog
)

if(WIN32)
    target_compile_definitions(test_trajectory PRIVATE
        NOMINMAX
    )
    target_compile_options(test_trajectory PRIVATE /utf-8)
endif()

gtest_discover_tests(test_trajectory)

# ============================================================================
# Firmware Tests
# ============================================================================
add_executable(test_firmware
    tests/test_firmware.cpp
)

target_include_directories(test_firmware
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(test_firmware
    PRIVATE
        GTest::gtest
        GTest::gtest_main
        Boost::system
)

if(WIN32)
    target_compile_definitions(test_firmware PRIVATE
        NOMINMAX
        _WIN32_WINNT=0x0A00
    )
endif()

gtest_discover_tests(test_firmware)

# ============================================================================
# Binary Protocol Tests (V2 packet framing, CRC, struct sizes)
# ============================================================================
add_executable(test_binary_protocol
    tests/test_binary_protocol.cpp
)

target_include_directories(test_binary_protocol
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(test_binary_protocol
    PRIVATE
        GTest::gtest
        GTest::gtest_main
)

if(WIN32)
    target_compile_definitions(test_binary_protocol PRIVATE
        NOMINMAX
        _WIN32_WINNT=0x0A00
    )
endif()

gtest_discover_tests(test_binary_protocol)

# ============================================================================
# Firmware V2 Tests (Simulator drive states, jog, homing, I/O)
# ============================================================================
add_executable(test_firmware_v2
    tests/test_firmware_v2.cpp
    src/firmware/FirmwareSimulator.cpp
    src/jog/RuckigSmoother.cpp
    src/logging/Logger.cpp
)

target_include_directories(test_firmware_v2
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(test_firmware_v2
    PRIVATE
        GTest::gtest
        GTest::gtest_main
        spdlog::spdlog
        Eigen3::Eigen
        Boost::system
        ruckig::ruckig
)

if(WIN32)
    target_compile_definitions(test_firmware_v2 PRIVATE
        NOMINMAX
        _WIN32_WINNT=0x0A00
        _USE_MATH_DEFINES
    )
endif()

gtest_discover_tests(test_firmware_v2)

# ============================================================================
# Welding Tests
# ============================================================================
add_executable(test_welding
    tests/test_welding.cpp
)

target_include_directories(test_welding
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(test_welding
    PRIVATE
        GTest::gtest
        GTest::gtest_main
)

if(WIN32)
    target_compile_definitions(test_welding PRIVATE
        NOMINMAX
    )
endif()

gtest_discover_tests(test_welding)

# ============================================================================
# Weaving Tests
# ============================================================================
add_executable(test_weaving
    tests/test_weaving.cpp
)

target_include_directories(test_weaving
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(test_weaving
    PRIVATE
        GTest::gtest
        GTest::gtest_main
        Eigen3::Eigen
)

if(WIN32)
    target_compile_definitions(test_weaving PRIVATE
        NOMINMAX
    )
endif()

gtest_discover_tests(test_weaving)

# ============================================================================
# Vision Tests
# ============================================================================
add_executable(test_vision
    tests/test_vision.cpp
    src/vision/HikrobotLaserProfiler.cpp
    src/vision/SensorManager.cpp
)

target_include_directories(test_vision
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(test_vision
    PRIVATE
        GTest::gtest
        GTest::gtest_main
        spdlog::spdlog
        nlohmann_json::nlohmann_json
)

if(WIN32)
    target_compile_definitions(test_vision PRIVATE
        NOMINMAX
    )
endif()

gtest_discover_tests(test_vision)

# ============================================================================
# Seam Detection Tests
# ============================================================================
add_executable(test_seam
    tests/test_seam.cpp
    src/vision/ProfileProcessor.cpp
    src/vision/JointDetector.cpp
    src/vision/SeamTracker.cpp
)

target_include_directories(test_seam
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(test_seam
    PRIVATE
        GTest::gtest
        GTest::gtest_main
        spdlog::spdlog
)

if(WIN32)
    target_compile_definitions(test_seam PRIVATE
        NOMINMAX
    )
endif()

gtest_discover_tests(test_seam)

# ============================================================================
# Safety Tests
# ============================================================================
add_executable(test_safety
    tests/test_safety.cpp
    src/logging/Logger.cpp
    src/state/StateMachine.cpp
)

target_include_directories(test_safety
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(test_safety
    PRIVATE
        GTest::gtest
        GTest::gtest_main
        spdlog::spdlog
)

if(WIN32)
    target_compile_definitions(test_safety PRIVATE
        NOMINMAX
    )
endif()

gtest_discover_tests(test_safety)

# ============================================================================
# Motion Controller Tests
# ============================================================================
add_executable(test_motion_controller
    tests/test_motion_controller.cpp
    src/logging/Logger.cpp
)

target_include_directories(test_motion_controller
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(test_motion_controller
    PRIVATE
        GTest::gtest
        GTest::gtest_main
        spdlog::spdlog
        Eigen3::Eigen
)

if(WIN32)
    target_compile_definitions(test_motion_controller PRIVATE
        NOMINMAX
    )
endif()

gtest_discover_tests(test_motion_controller)

# ============================================================================
# Core Platform Tests
# ============================================================================
add_executable(test_core_platform
    tests/test_core_platform.cpp
    src/config/ConfigManager.cpp
    src/config/RobotCatalog.cpp
    src/config/RobotInstance.cpp
    src/logging/Logger.cpp
    src/ipc/IpcServer.cpp
)

target_include_directories(test_core_platform
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(test_core_platform
    PRIVATE
        GTest::gtest
        GTest::gtest_main
        spdlog::spdlog
        nlohmann_json::nlohmann_json
        cppzmq
        yaml-cpp::yaml-cpp
)

if(WIN32)
    target_compile_definitions(test_core_platform PRIVATE
        NOMINMAX
    )
endif()

gtest_discover_tests(test_core_platform)

# ============================================================================
# Feature Registry Tests (All-in-one verification)
# ============================================================================
add_executable(test_feature_registry
    tests/test_feature_registry.cpp
)

target_include_directories(test_feature_registry
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(test_feature_registry
    PRIVATE
        GTest::gtest
        GTest::gtest_main
)

if(WIN32)
    target_compile_definitions(test_feature_registry PRIVATE
        NOMINMAX
    )
endif()

gtest_discover_tests(test_feature_registry)

# ============================================================================
# IK Debug Test (standalone, no GTest)
# ============================================================================
add_executable(test_ik_debug
    ${CMAKE_CURRENT_SOURCE_DIR}/../../test_ik_debug.cpp
)

target_include_directories(test_ik_debug
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../..
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(test_ik_debug
    PRIVATE
        Eigen3::Eigen
)

if(WIN32)
    target_compile_definitions(test_ik_debug PRIVATE
        NOMINMAX
    )
endif()

# IK Wrist debug
add_executable(test_ik_wrist
    ${CMAKE_CURRENT_SOURCE_DIR}/../../test_ik_wrist.cpp
)

target_include_directories(test_ik_wrist
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../..
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(test_ik_wrist
    PRIVATE
        Eigen3::Eigen
)

if(WIN32)
    target_compile_definitions(test_ik_wrist PRIVATE
        NOMINMAX
    )
endif()

# IK Debug 3
add_executable(test_ik_debug3
    ${CMAKE_CURRENT_SOURCE_DIR}/../../test_ik_debug3.cpp
)

target_include_directories(test_ik_debug3
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../..
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(test_ik_debug3
    PRIVATE
        Eigen3::Eigen
)

if(WIN32)
    target_compile_definitions(test_ik_debug3 PRIVATE
        NOMINMAX
    )
endif()

# IK Debug 2
add_executable(test_ik_debug2
    ${CMAKE_CURRENT_SOURCE_DIR}/../../test_ik_debug2.cpp
)

target_include_directories(test_ik_debug2
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../..
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(test_ik_debug2
    PRIVATE
        Eigen3::Eigen
)

if(WIN32)
    target_compile_definitions(test_ik_debug2 PRIVATE
        NOMINMAX
    )
endif()

# Quick IK test
add_executable(test_ik_quick
    ${CMAKE_CURRENT_SOURCE_DIR}/../../test_ik_quick.cpp
)

target_include_directories(test_ik_quick
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../..
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(test_ik_quick
    PRIVATE
        Eigen3::Eigen
)

if(WIN32)
    target_compile_definitions(test_ik_quick PRIVATE
        NOMINMAX
    )
endif()

add_executable(test_ik_yjog
    ${CMAKE_CURRENT_SOURCE_DIR}/../../test_ik_yjog.cpp
)

target_include_directories(test_ik_yjog
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../..
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(test_ik_yjog
    PRIVATE
        Eigen3::Eigen
)

if(WIN32)
    target_compile_definitions(test_ik_yjog PRIVATE
        NOMINMAX
    )
endif()

add_executable(test_r36_verify
    ${CMAKE_CURRENT_SOURCE_DIR}/../../test_r36_verify.cpp
)

target_include_directories(test_r36_verify
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../..
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(test_r36_verify
    PRIVATE
        Eigen3::Eigen
)

if(WIN32)
    target_compile_definitions(test_r36_verify PRIVATE
        NOMINMAX
    )
endif()

add_executable(test_ik_reject
    ${CMAKE_CURRENT_SOURCE_DIR}/../../test_ik_reject.cpp
)

target_include_directories(test_ik_reject
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../..
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(test_ik_reject
    PRIVATE
        Eigen3::Eigen
)

if(WIN32)
    target_compile_definitions(test_ik_reject PRIVATE
        NOMINMAX
    )
endif()

# Tool Jog Z-Drift diagnostic
add_executable(test_tool_jog_zdrift
    tests/test_tool_jog_zdrift.cpp
    src/kinematics/UrdfForwardKinematics.cpp
    src/tool/ToolTypes.cpp
    src/logging/Logger.cpp
)

target_include_directories(test_tool_jog_zdrift
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(test_tool_jog_zdrift
    PRIVATE
        Eigen3::Eigen
        spdlog::spdlog
)

if(WIN32)
    target_compile_definitions(test_tool_jog_zdrift PRIVATE NOMINMAX)
    target_compile_options(test_tool_jog_zdrift PRIVATE /utf-8)
endif()

# ============================================================================
# Post-Build: Copy Config Files
# ============================================================================
# Copy config folder to build output directory after each build
add_custom_command(TARGET robot_core POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/../config"
        "$<TARGET_FILE_DIR:robot_core>/../config"
    COMMENT "Copying config files to build output..."
)

# ============================================================================
# Installation
# ============================================================================
install(TARGETS robot_core
    RUNTIME DESTINATION bin
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../config/
    DESTINATION config
)

# ============================================================================
# Summary
# ============================================================================
message(STATUS "")
message(STATUS "=== Robot Controller Core ===")
message(STATUS "Version:      ${PROJECT_VERSION}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type:   ${CMAKE_BUILD_TYPE}")
message(STATUS "")
