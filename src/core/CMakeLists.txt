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
