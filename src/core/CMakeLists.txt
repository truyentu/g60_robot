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

# ============================================================================
# Source Files
# ============================================================================
set(CORE_SOURCES
    src/main.cpp
    src/ipc/IpcServer.cpp
    src/config/ConfigManager.cpp
    src/robot/RobotModel.cpp
    src/logging/Logger.cpp
    src/state/StateMachine.cpp
    src/controller/RobotController.cpp
)

set(CORE_HEADERS
    include/robot_controller/core.hpp
    src/ipc/IpcServer.hpp
    src/ipc/Message.hpp
    src/ipc/MessageTypes.hpp
    src/config/ConfigManager.hpp
    src/config/RobotConfig.hpp
    src/config/SystemConfig.hpp
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
    src/trajectory/TrajectoryTypes.hpp
    src/trajectory/VelocityProfile.hpp
    src/trajectory/Interpolators.hpp
    src/trajectory/TrajectoryPlanner.hpp
    src/trajectory/TrajectoryExecutor.hpp
    src/firmware/FirmwareProtocol.hpp
    src/firmware/SerialPort.hpp
    src/firmware/FirmwareInterface.hpp
    src/firmware/MotionStreamer.hpp
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
)

if(WIN32)
    target_compile_definitions(test_kinematics PRIVATE
        NOMINMAX
    )
endif()

gtest_discover_tests(test_kinematics)

# ============================================================================
# Trajectory Tests
# ============================================================================
add_executable(test_trajectory
    tests/test_trajectory.cpp
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
)

if(WIN32)
    target_compile_definitions(test_trajectory PRIVATE
        NOMINMAX
    )
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
