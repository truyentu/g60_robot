# PHASE 1: FOUNDATION

## Document Info
| Item | Value |
|------|-------|
| **Phase** | 1 - Foundation |
| **Status** | Planning |
| **Version** | 1.0 |
| **Last Updated** | 2026-02-01 |
| **Prerequisites** | None |
| **Next Phase** | Phase 2 - Motion Core |

---

## 1. PHASE OVERVIEW

### 1.1. Objective
Xây dựng nền tảng cơ bản cho toàn bộ hệ thống Robot Controller, bao gồm project structure, communication layer, configuration system, và HMI shell.

### 1.2. Scope

**In Scope:**
- Project structure cho C# và C++
- Build system (CMake + .NET)
- IPC layer giữa UI và Core
- Configuration management
- Logging framework
- Basic HMI shell với 3D viewport
- Robot model loader

**Out of Scope:**
- Motion control logic
- State machine implementation
- Welding/Vision modules
- Hardware communication

### 1.3. Milestone: "Hello Robot"

Phase 1 hoàn thành khi đạt được:
- [ ] C# UI khởi động thành công
- [ ] Hiển thị mô hình robot 3D trong viewport
- [ ] C++ Core process chạy độc lập
- [ ] UI và Core giao tiếp được qua IPC
- [ ] Config load được từ file YAML/JSON
- [ ] Logs ghi được vào file và console

---

## 2. ARCHITECTURE

### 2.1. Phase 1 Components

```
┌─────────────────────────────────────────────────────────────────┐
│                      PHASE 1 SCOPE                               │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  UI LAYER (C# WPF)                                         │ │
│  │                                                             │ │
│  │  ┌───────────┐  ┌───────────┐  ┌───────────┐              │ │
│  │  │  MainWin  │  │  3D View  │  │  Config   │              │ │
│  │  │  Shell    │  │  Helix    │  │  Manager  │              │ │
│  │  └───────────┘  └───────────┘  └───────────┘              │ │
│  │        │              │              │                     │ │
│  │        └──────────────┴──────────────┘                     │ │
│  │                       │                                     │ │
│  │              ┌────────┴────────┐                           │ │
│  │              │  IPC Client     │                           │ │
│  │              │  (NetMQ)        │                           │ │
│  │              └────────┬────────┘                           │ │
│  └───────────────────────┼────────────────────────────────────┘ │
│                          │                                       │
│                    ZeroMQ TCP                                    │
│                          │                                       │
│  ┌───────────────────────┼────────────────────────────────────┐ │
│  │  CORE LAYER (C++)     │                                    │ │
│  │              ┌────────┴────────┐                           │ │
│  │              │  IPC Server     │                           │ │
│  │              │  (cppzmq)       │                           │ │
│  │              └────────┬────────┘                           │ │
│  │                       │                                     │ │
│  │        ┌──────────────┴──────────────┐                     │ │
│  │        │              │              │                     │ │
│  │  ┌───────────┐  ┌───────────┐  ┌───────────┐              │ │
│  │  │  Config   │  │  Logger   │  │  Robot    │              │ │
│  │  │  Loader   │  │  spdlog   │  │  Model    │              │ │
│  │  └───────────┘  └───────────┘  └───────────┘              │ │
│  │                                                             │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2. Folder Structure

```
📁 Robot_controller/
├── 📁 src/
│   ├── 📁 ui/                              # C# WPF Application
│   │   ├── 📁 RobotController.UI/          # Main UI project
│   │   │   ├── 📁 Views/
│   │   │   │   ├── MainWindow.xaml
│   │   │   │   ├── Viewport3DView.xaml
│   │   │   │   └── ConfigView.xaml
│   │   │   ├── 📁 ViewModels/
│   │   │   │   ├── MainViewModel.cs
│   │   │   │   ├── Viewport3DViewModel.cs
│   │   │   │   └── ConfigViewModel.cs
│   │   │   ├── 📁 Services/
│   │   │   │   ├── IpcClientService.cs
│   │   │   │   ├── ConfigService.cs
│   │   │   │   └── RobotModelService.cs
│   │   │   ├── 📁 Models/
│   │   │   │   ├── RobotModel.cs
│   │   │   │   └── JointState.cs
│   │   │   ├── App.xaml
│   │   │   └── RobotController.UI.csproj
│   │   │
│   │   ├── 📁 RobotController.Common/      # Shared library
│   │   │   ├── 📁 Messages/
│   │   │   │   ├── IpcMessage.cs
│   │   │   │   └── MessageTypes.cs
│   │   │   └── RobotController.Common.csproj
│   │   │
│   │   └── RobotController.sln
│   │
│   ├── 📁 core/                            # C++ Core
│   │   ├── 📁 src/
│   │   │   ├── 📁 ipc/
│   │   │   │   ├── IpcServer.hpp
│   │   │   │   ├── IpcServer.cpp
│   │   │   │   ├── Message.hpp
│   │   │   │   └── MessageHandler.cpp
│   │   │   ├── 📁 config/
│   │   │   │   ├── ConfigManager.hpp
│   │   │   │   ├── ConfigManager.cpp
│   │   │   │   └── RobotConfig.hpp
│   │   │   ├── 📁 robot/
│   │   │   │   ├── RobotModel.hpp
│   │   │   │   ├── RobotModel.cpp
│   │   │   │   └── DHParameters.hpp
│   │   │   ├── 📁 logging/
│   │   │   │   └── Logger.hpp
│   │   │   └── main.cpp
│   │   │
│   │   ├── 📁 include/                     # Public headers
│   │   │   └── robot_controller/
│   │   │       └── core.hpp
│   │   │
│   │   ├── 📁 third_party/                 # External libs
│   │   │   ├── spdlog/
│   │   │   ├── nlohmann_json/
│   │   │   └── cppzmq/
│   │   │
│   │   ├── 📁 tests/                       # Unit tests
│   │   │   ├── test_config.cpp
│   │   │   └── test_ipc.cpp
│   │   │
│   │   └── CMakeLists.txt
│   │
│   └── 📁 firmware/                        # Teensy (Phase 2+)
│       └── (empty for Phase 1)
│
├── 📁 config/                              # Configuration files
│   ├── robot_config.yaml                   # Robot parameters
│   ├── system_config.yaml                  # System settings
│   └── ui_config.json                      # UI preferences
│
├── 📁 resources/                           # Resource files
│   ├── 📁 models/                          # 3D models
│   │   ├── robot_base.stl
│   │   ├── robot_link1.stl
│   │   ├── robot_link2.stl
│   │   ├── robot_link3.stl
│   │   ├── robot_link4.stl
│   │   ├── robot_link5.stl
│   │   └── robot_link6.stl
│   └── 📁 icons/
│
├── 📁 docs/                                # Documentation
│
└── 📁 scripts/                             # Build/utility scripts
    ├── build_all.ps1
    ├── build_core.ps1
    └── build_ui.ps1
```

---

## 3. TASK BREAKDOWN

### 3.1. Task Summary

| ID | Task | Priority | Status |
|----|------|----------|--------|
| P1-01 | Project Structure Setup | P0 | Todo |
| P1-02 | C++ Build System (CMake) | P0 | Todo |
| P1-03 | C# Solution Setup | P0 | Todo |
| P1-04 | IPC Protocol Definition | P0 | Todo |
| P1-05 | IPC Server (C++) | P0 | Todo |
| P1-06 | IPC Client (C#) | P0 | Todo |
| P1-07 | Config System (C++) | P1 | Todo |
| P1-08 | Config System (C#) | P1 | Todo |
| P1-09 | Logging Framework (C++) | P1 | Todo |
| P1-10 | Logging Framework (C#) | P1 | Todo |
| P1-11 | HMI Shell Window | P1 | Todo |
| P1-12 | 3D Viewport Setup | P1 | Todo |
| P1-13 | Robot Model Loader | P1 | Todo |
| P1-14 | Robot 3D Visualization | P1 | Todo |
| P1-15 | Integration Testing | P2 | Todo |

### 3.2. Task Details

---

#### P1-01: Project Structure Setup

| Attribute | Value |
|-----------|-------|
| **Priority** | P0 - Critical |
| **Dependencies** | None |
| **Estimated Effort** | Small |

**Description:**
Tạo folder structure cơ bản cho toàn bộ project theo cấu trúc đã định nghĩa.

**Acceptance Criteria:**
- [ ] Tạo đầy đủ folder structure như Section 2.2
- [ ] Tạo README.md ở root
- [ ] Tạo .gitignore phù hợp
- [ ] Khởi tạo git repository

**Deliverables:**
- Folder structure hoàn chỉnh
- README.md
- .gitignore

---

#### P1-02: C++ Build System (CMake)

| Attribute | Value |
|-----------|-------|
| **Priority** | P0 - Critical |
| **Dependencies** | P1-01 |
| **Estimated Effort** | Medium |

**Description:**
Setup CMake build system cho C++ Core, bao gồm việc tích hợp các third-party libraries.

**Tech Stack:**
- CMake 3.20+
- MSVC 2022
- vcpkg hoặc manual third-party management

**CMakeLists.txt Structure:**
```cmake
cmake_minimum_required(VERSION 3.20)
project(RobotControllerCore VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Third-party libraries
add_subdirectory(third_party/spdlog)
find_package(cppzmq REQUIRED)
find_package(nlohmann_json REQUIRED)

# Main executable
add_executable(robot_core
    src/main.cpp
    src/ipc/IpcServer.cpp
    src/config/ConfigManager.cpp
    src/robot/RobotModel.cpp
)

target_link_libraries(robot_core
    PRIVATE
        spdlog::spdlog
        cppzmq
        nlohmann_json::nlohmann_json
)

# Tests
enable_testing()
add_subdirectory(tests)
```

**Acceptance Criteria:**
- [ ] CMake configure thành công
- [ ] Build ra executable `robot_core.exe`
- [ ] Third-party libs link đúng
- [ ] Debug và Release configuration hoạt động

**Deliverables:**
- CMakeLists.txt
- Build scripts (build_core.ps1)
- Compiled executable

---

#### P1-03: C# Solution Setup

| Attribute | Value |
|-----------|-------|
| **Priority** | P0 - Critical |
| **Dependencies** | P1-01 |
| **Estimated Effort** | Medium |

**Description:**
Setup .NET solution cho UI layer với WPF và các dependencies cần thiết.

**Tech Stack:**
- .NET 6.0 hoặc .NET 8.0
- WPF
- Helix Toolkit WPF
- NetMQ
- CommunityToolkit.Mvvm
- Serilog

**Project Structure:**
```xml
<!-- RobotController.UI.csproj -->
<Project Sdk="Microsoft.NET.Sdk">
  <PropertyGroup>
    <OutputType>WinExe</OutputType>
    <TargetFramework>net6.0-windows</TargetFramework>
    <UseWPF>true</UseWPF>
    <Nullable>enable</Nullable>
  </PropertyGroup>

  <ItemGroup>
    <PackageReference Include="HelixToolkit.Wpf" Version="2.24.0" />
    <PackageReference Include="NetMQ" Version="4.0.1.13" />
    <PackageReference Include="CommunityToolkit.Mvvm" Version="8.2.2" />
    <PackageReference Include="Serilog" Version="3.1.1" />
    <PackageReference Include="Serilog.Sinks.File" Version="5.0.0" />
    <PackageReference Include="Serilog.Sinks.Console" Version="5.0.1" />
  </ItemGroup>

  <ItemGroup>
    <ProjectReference Include="..\RobotController.Common\RobotController.Common.csproj" />
  </ItemGroup>
</Project>
```

**Acceptance Criteria:**
- [ ] Solution build thành công
- [ ] WPF window khởi động được
- [ ] Tất cả NuGet packages restore đúng
- [ ] MVVM pattern setup đúng

**Deliverables:**
- RobotController.sln
- RobotController.UI.csproj
- RobotController.Common.csproj
- Basic MainWindow.xaml

---

#### P1-04: IPC Protocol Definition

| Attribute | Value |
|-----------|-------|
| **Priority** | P0 - Critical |
| **Dependencies** | P1-01 |
| **Estimated Effort** | Medium |

**Description:**
Định nghĩa protocol giao tiếp giữa UI (C#) và Core (C++) qua ZeroMQ.

**Design Decisions:**

| Aspect | Decision | Reason |
|--------|----------|--------|
| Transport | TCP | Đơn giản, debug dễ, có thể chạy UI và Core trên máy khác |
| Pattern | REQ-REP + PUB-SUB | REQ-REP cho commands, PUB-SUB cho status updates |
| Serialization | JSON | Human-readable, debug dễ, đủ nhanh cho use case này |
| Port | 5555 (REQ-REP), 5556 (PUB-SUB) | Default ports |

**Message Format:**
```json
{
  "type": "string",        // Message type
  "id": "string",          // Unique message ID (UUID)
  "timestamp": "int64",    // Unix timestamp ms
  "payload": { }           // Type-specific data
}
```

**Message Types:**

| Type | Direction | Description |
|------|-----------|-------------|
| `PING` | UI → Core | Health check |
| `PONG` | Core → UI | Health check response |
| `GET_STATUS` | UI → Core | Request system status |
| `STATUS` | Core → UI | System status response |
| `GET_JOINT_POSITIONS` | UI → Core | Request current joint angles |
| `JOINT_POSITIONS` | Core → UI | Joint angles response |
| `GET_CONFIG` | UI → Core | Request configuration |
| `CONFIG` | Core → UI | Configuration response |
| `SET_CONFIG` | UI → Core | Update configuration |
| `CONFIG_ACK` | Core → UI | Configuration update acknowledgment |
| `ERROR` | Core → UI | Error notification |

**Example Messages:**

```json
// PING
{
  "type": "PING",
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": 1706745600000,
  "payload": {}
}

// PONG
{
  "type": "PONG",
  "id": "550e8400-e29b-41d4-a716-446655440001",
  "timestamp": 1706745600005,
  "payload": {
    "core_version": "1.0.0",
    "uptime_ms": 12345
  }
}

// JOINT_POSITIONS
{
  "type": "JOINT_POSITIONS",
  "id": "550e8400-e29b-41d4-a716-446655440002",
  "timestamp": 1706745600010,
  "payload": {
    "joints": [0.0, -45.0, 90.0, 0.0, 45.0, 0.0],
    "unit": "degrees"
  }
}

// STATUS (Published periodically)
{
  "type": "STATUS",
  "id": "550e8400-e29b-41d4-a716-446655440003",
  "timestamp": 1706745600015,
  "payload": {
    "state": "IDLE",
    "mode": "MANUAL",
    "joints": [0.0, -45.0, 90.0, 0.0, 45.0, 0.0],
    "tcp_position": [500.0, 0.0, 600.0, 0.0, 180.0, 0.0],
    "errors": []
  }
}
```

**Acceptance Criteria:**
- [ ] Protocol document hoàn chỉnh
- [ ] Message schema định nghĩa rõ ràng
- [ ] C++ và C# message classes tương thích

**Deliverables:**
- Protocol specification document
- Message.hpp (C++)
- IpcMessage.cs (C#)
- MessageTypes enum

---

#### P1-05: IPC Server (C++)

| Attribute | Value |
|-----------|-------|
| **Priority** | P0 - Critical |
| **Dependencies** | P1-02, P1-04 |
| **Estimated Effort** | Large |

**Description:**
Implement IPC server trong C++ Core sử dụng ZeroMQ (cppzmq).

**Class Design:**

```cpp
// IpcServer.hpp
#pragma once

#include <zmq.hpp>
#include <string>
#include <functional>
#include <thread>
#include <atomic>
#include <nlohmann/json.hpp>

namespace robot_controller {

using json = nlohmann::json;
using MessageHandler = std::function<json(const json&)>;

class IpcServer {
public:
    IpcServer(const std::string& req_address = "tcp://*:5555",
              const std::string& pub_address = "tcp://*:5556");
    ~IpcServer();

    // Start/Stop server
    void start();
    void stop();
    bool isRunning() const;

    // Register message handlers
    void registerHandler(const std::string& message_type, MessageHandler handler);

    // Publish status updates
    void publish(const json& message);

private:
    void reqRepLoop();      // REQ-REP thread
    void handleMessage(const std::string& raw_message);

    zmq::context_t m_context;
    zmq::socket_t m_rep_socket;     // REQ-REP socket
    zmq::socket_t m_pub_socket;     // PUB socket

    std::thread m_req_thread;
    std::atomic<bool> m_running{false};

    std::unordered_map<std::string, MessageHandler> m_handlers;
    std::mutex m_handlers_mutex;
};

} // namespace robot_controller
```

**Implementation Notes:**
- REQ-REP socket chạy trên thread riêng
- PUB socket publish từ main thread hoặc timer
- Thread-safe message handling
- Graceful shutdown

**Acceptance Criteria:**
- [ ] Server khởi động và bind đúng ports
- [ ] Nhận và xử lý được PING message
- [ ] Trả về PONG response
- [ ] Publish được STATUS messages
- [ ] Graceful shutdown không memory leak

**Deliverables:**
- IpcServer.hpp
- IpcServer.cpp
- Unit tests

---

#### P1-06: IPC Client (C#)

| Attribute | Value |
|-----------|-------|
| **Priority** | P0 - Critical |
| **Dependencies** | P1-03, P1-04 |
| **Estimated Effort** | Large |

**Description:**
Implement IPC client trong C# UI sử dụng NetMQ.

**Class Design:**

```csharp
// IpcClientService.cs
using NetMQ;
using NetMQ.Sockets;
using System.Text.Json;

namespace RobotController.UI.Services;

public interface IIpcClientService : IDisposable
{
    bool IsConnected { get; }

    Task<bool> ConnectAsync(string reqAddress, string subAddress);
    void Disconnect();

    Task<TResponse?> SendRequestAsync<TRequest, TResponse>(TRequest request)
        where TRequest : IpcMessage
        where TResponse : IpcMessage;

    event EventHandler<StatusMessage>? StatusReceived;
}

public class IpcClientService : IIpcClientService
{
    private RequestSocket? _reqSocket;
    private SubscriberSocket? _subSocket;
    private NetMQPoller? _poller;
    private Task? _pollerTask;
    private bool _isConnected;

    public bool IsConnected => _isConnected;

    public event EventHandler<StatusMessage>? StatusReceived;

    public async Task<bool> ConnectAsync(string reqAddress, string subAddress)
    {
        try
        {
            _reqSocket = new RequestSocket();
            _reqSocket.Connect(reqAddress);

            _subSocket = new SubscriberSocket();
            _subSocket.Connect(subAddress);
            _subSocket.SubscribeToAnyTopic();

            _subSocket.ReceiveReady += OnSubscriberReceiveReady;

            _poller = new NetMQPoller { _subSocket };
            _pollerTask = Task.Run(() => _poller.Run());

            // Test connection with PING
            var pong = await SendRequestAsync<PingMessage, PongMessage>(new PingMessage());
            _isConnected = pong != null;

            return _isConnected;
        }
        catch (Exception ex)
        {
            // Log error
            return false;
        }
    }

    public async Task<TResponse?> SendRequestAsync<TRequest, TResponse>(TRequest request)
        where TRequest : IpcMessage
        where TResponse : IpcMessage
    {
        if (_reqSocket == null) return default;

        var json = JsonSerializer.Serialize(request);
        _reqSocket.SendFrame(json);

        var responseJson = _reqSocket.ReceiveFrameString();
        return JsonSerializer.Deserialize<TResponse>(responseJson);
    }

    private void OnSubscriberReceiveReady(object? sender, NetMQSocketEventArgs e)
    {
        var message = e.Socket.ReceiveFrameString();
        var status = JsonSerializer.Deserialize<StatusMessage>(message);
        if (status != null)
        {
            StatusReceived?.Invoke(this, status);
        }
    }

    public void Disconnect()
    {
        _poller?.Stop();
        _subSocket?.Dispose();
        _reqSocket?.Dispose();
        _isConnected = false;
    }

    public void Dispose()
    {
        Disconnect();
        _poller?.Dispose();
    }
}
```

**Acceptance Criteria:**
- [ ] Connect được tới C++ server
- [ ] Gửi PING nhận PONG thành công
- [ ] Subscribe và nhận được STATUS updates
- [ ] Reconnect khi mất kết nối
- [ ] Proper disposal và cleanup

**Deliverables:**
- IIpcClientService.cs
- IpcClientService.cs
- Message classes
- Unit tests

---

#### P1-07: Config System (C++)

| Attribute | Value |
|-----------|-------|
| **Priority** | P1 - High |
| **Dependencies** | P1-02 |
| **Estimated Effort** | Medium |

**Description:**
Implement configuration loading và management trong C++ Core.

**Config File Format (YAML):**

```yaml
# robot_config.yaml
robot:
  name: "Robot6DOF"
  type: "6-axis-articulated"

  # DH Parameters (Modified DH Convention)
  dh_parameters:
    - { a: 0,     alpha: 0,      d: 400,  theta_offset: 0 }      # Joint 1
    - { a: 25,    alpha: -90,    d: 0,    theta_offset: -90 }    # Joint 2
    - { a: 560,   alpha: 0,      d: 0,    theta_offset: 0 }      # Joint 3
    - { a: 35,    alpha: -90,    d: 515,  theta_offset: 0 }      # Joint 4
    - { a: 0,     alpha: 90,     d: 0,    theta_offset: 0 }      # Joint 5
    - { a: 0,     alpha: -90,    d: 80,   theta_offset: 0 }      # Joint 6

  # Joint limits (degrees)
  joint_limits:
    - { min: -170, max: 170, max_velocity: 120, max_acceleration: 500 }
    - { min: -120, max: 60,  max_velocity: 120, max_acceleration: 500 }
    - { min: -80,  max: 165, max_velocity: 120, max_acceleration: 500 }
    - { min: -185, max: 185, max_velocity: 190, max_acceleration: 800 }
    - { min: -120, max: 120, max_velocity: 190, max_acceleration: 800 }
    - { min: -360, max: 360, max_velocity: 250, max_acceleration: 1000 }

  # TCP offset from flange
  tcp_offset:
    x: 0
    y: 0
    z: 150
    rx: 0
    ry: 0
    rz: 0

# system_config.yaml
system:
  ipc:
    req_port: 5555
    pub_port: 5556

  logging:
    level: "debug"          # trace, debug, info, warn, error
    file: "logs/core.log"
    max_size_mb: 10
    max_files: 5

  control:
    cycle_time_ms: 1
    interpolation_rate_hz: 1000
```

**Class Design:**

```cpp
// ConfigManager.hpp
#pragma once

#include <string>
#include <optional>
#include <nlohmann/json.hpp>

namespace robot_controller {

struct DHParameter {
    double a;           // Link length
    double alpha;       // Link twist (radians)
    double d;           // Link offset
    double theta_offset; // Joint angle offset
};

struct JointLimit {
    double min;             // degrees
    double max;             // degrees
    double max_velocity;    // deg/s
    double max_acceleration; // deg/s^2
};

struct RobotConfig {
    std::string name;
    std::string type;
    std::vector<DHParameter> dh_parameters;
    std::vector<JointLimit> joint_limits;
    std::array<double, 6> tcp_offset;
};

struct SystemConfig {
    int req_port = 5555;
    int pub_port = 5556;
    std::string log_level = "info";
    std::string log_file = "logs/core.log";
    double cycle_time_ms = 1.0;
};

class ConfigManager {
public:
    static ConfigManager& instance();

    bool loadRobotConfig(const std::string& filepath);
    bool loadSystemConfig(const std::string& filepath);

    const RobotConfig& robotConfig() const { return m_robot_config; }
    const SystemConfig& systemConfig() const { return m_system_config; }

    bool saveRobotConfig(const std::string& filepath) const;

private:
    ConfigManager() = default;

    RobotConfig m_robot_config;
    SystemConfig m_system_config;
};

} // namespace robot_controller
```

**Acceptance Criteria:**
- [ ] Load robot_config.yaml thành công
- [ ] Load system_config.yaml thành công
- [ ] Parse DH parameters đúng
- [ ] Parse joint limits đúng
- [ ] Error handling cho file không tồn tại hoặc format sai

**Deliverables:**
- ConfigManager.hpp
- ConfigManager.cpp
- RobotConfig.hpp
- Sample config files
- Unit tests

---

#### P1-08: Config System (C#)

| Attribute | Value |
|-----------|-------|
| **Priority** | P1 - High |
| **Dependencies** | P1-03 |
| **Estimated Effort** | Small |

**Description:**
Implement configuration loading trong C# UI cho UI-specific settings.

**Config File Format (JSON):**

```json
{
  "ui": {
    "theme": "dark",
    "language": "vi-VN",
    "viewport": {
      "background_color": "#1E1E1E",
      "grid_visible": true,
      "axis_visible": true
    },
    "connection": {
      "core_address": "tcp://localhost:5555",
      "pub_address": "tcp://localhost:5556",
      "auto_connect": true,
      "reconnect_interval_ms": 5000
    }
  },
  "recent_files": [],
  "window": {
    "width": 1920,
    "height": 1080,
    "maximized": false
  }
}
```

**Acceptance Criteria:**
- [ ] Load ui_config.json thành công
- [ ] Save config khi thay đổi
- [ ] Default values khi file không tồn tại

**Deliverables:**
- ConfigService.cs
- UiConfig.cs
- Default config file

---

#### P1-09: Logging Framework (C++)

| Attribute | Value |
|-----------|-------|
| **Priority** | P1 - High |
| **Dependencies** | P1-02, P1-07 |
| **Estimated Effort** | Small |

**Description:**
Setup logging framework sử dụng spdlog.

**Class Design:**

```cpp
// Logger.hpp
#pragma once

#include <spdlog/spdlog.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace robot_controller {

class Logger {
public:
    static void init(const std::string& log_file,
                     const std::string& level = "info",
                     size_t max_size = 10 * 1024 * 1024,
                     size_t max_files = 5);

    static std::shared_ptr<spdlog::logger> get() { return s_logger; }

private:
    static std::shared_ptr<spdlog::logger> s_logger;
};

// Convenience macros
#define LOG_TRACE(...) robot_controller::Logger::get()->trace(__VA_ARGS__)
#define LOG_DEBUG(...) robot_controller::Logger::get()->debug(__VA_ARGS__)
#define LOG_INFO(...)  robot_controller::Logger::get()->info(__VA_ARGS__)
#define LOG_WARN(...)  robot_controller::Logger::get()->warn(__VA_ARGS__)
#define LOG_ERROR(...) robot_controller::Logger::get()->error(__VA_ARGS__)

} // namespace robot_controller
```

**Acceptance Criteria:**
- [ ] Logs ghi vào console với màu sắc
- [ ] Logs ghi vào file với rotation
- [ ] Log level configurable
- [ ] Thread-safe

**Deliverables:**
- Logger.hpp
- Logger.cpp

---

#### P1-10: Logging Framework (C#)

| Attribute | Value |
|-----------|-------|
| **Priority** | P1 - High |
| **Dependencies** | P1-03, P1-08 |
| **Estimated Effort** | Small |

**Description:**
Setup logging framework sử dụng Serilog.

**Setup:**

```csharp
// Program.cs or App.xaml.cs
Log.Logger = new LoggerConfiguration()
    .MinimumLevel.Debug()
    .WriteTo.Console()
    .WriteTo.File("logs/ui.log",
        rollingInterval: RollingInterval.Day,
        retainedFileCountLimit: 7)
    .CreateLogger();
```

**Acceptance Criteria:**
- [ ] Logs ghi vào console
- [ ] Logs ghi vào file với daily rotation
- [ ] Structured logging hoạt động

**Deliverables:**
- Logging configuration
- Log helper class (optional)

---

#### P1-11: HMI Shell Window

| Attribute | Value |
|-----------|-------|
| **Priority** | P1 - High |
| **Dependencies** | P1-03, P1-08, P1-10 |
| **Estimated Effort** | Medium |

**Description:**
Tạo main window shell với layout cơ bản cho HMI.

**Layout Design:**

```
┌─────────────────────────────────────────────────────────────────┐
│  Menu Bar                                                        │
├─────────────────────────────────────────────────────────────────┤
│  Toolbar                                                         │
├──────────────────────┬──────────────────────────────────────────┤
│                      │                                           │
│    Navigation        │           Main Content Area               │
│    Panel             │           (3D Viewport / Tabs)            │
│                      │                                           │
│    - Manual          │                                           │
│    - Program         │                                           │
│    - Config          │                                           │
│    - Diagnostics     │                                           │
│                      │                                           │
├──────────────────────┴──────────────────────────────────────────┤
│  Status Bar: [Connection Status] [Robot State] [Position]       │
└─────────────────────────────────────────────────────────────────┘
```

**Acceptance Criteria:**
- [ ] MainWindow với layout như design
- [ ] Navigation panel hoạt động
- [ ] Status bar hiển thị connection status
- [ ] Menu bar với basic items
- [ ] Responsive khi resize

**Deliverables:**
- MainWindow.xaml
- MainViewModel.cs
- Navigation service

---

#### P1-12: 3D Viewport Setup

| Attribute | Value |
|-----------|-------|
| **Priority** | P1 - High |
| **Dependencies** | P1-03, P1-11 |
| **Estimated Effort** | Medium |

**Description:**
Setup Helix Toolkit 3D viewport với camera control và grid.

**Features:**
- HelixViewport3D control
- Orbit camera (mouse drag)
- Pan (middle mouse)
- Zoom (scroll)
- Grid plane
- Coordinate axes (X-Red, Y-Green, Z-Blue)

**XAML Example:**

```xml
<helix:HelixViewport3D x:Name="Viewport"
                       ShowCoordinateSystem="True"
                       CoordinateSystemLabelForeground="White"
                       Background="{DynamicResource ViewportBackground}">

    <!-- Lighting -->
    <helix:DefaultLights/>

    <!-- Grid -->
    <helix:GridLinesVisual3D Width="2000" Length="2000"
                             MajorDistance="100" MinorDistance="25"
                             Thickness="0.5"/>

    <!-- Robot Model will be added here -->
    <ModelVisual3D x:Name="RobotModel"/>

</helix:HelixViewport3D>
```

**Acceptance Criteria:**
- [ ] Viewport hiển thị với grid
- [ ] Camera orbit/pan/zoom hoạt động
- [ ] Coordinate axes hiển thị đúng
- [ ] Background color configurable
- [ ] FPS ổn định (>30 FPS)

**Deliverables:**
- Viewport3DView.xaml
- Viewport3DViewModel.cs
- Camera controller

---

#### P1-13: Robot Model Loader

| Attribute | Value |
|-----------|-------|
| **Priority** | P1 - High |
| **Dependencies** | P1-07, P1-08 |
| **Estimated Effort** | Medium |

**Description:**
Load robot link meshes từ STL files và build kinematic chain.

**Data Structure:**

```csharp
public class RobotLink
{
    public string Name { get; set; }
    public int JointIndex { get; set; }
    public MeshGeometry3D Mesh { get; set; }
    public Transform3DGroup Transform { get; set; }
    public DHParameter DH { get; set; }
}

public class RobotModel
{
    public string Name { get; set; }
    public List<RobotLink> Links { get; set; }
    public double[] JointAngles { get; set; } // Current angles (radians)

    public void UpdateJointAngle(int jointIndex, double angleRad);
    public void UpdateAllJoints(double[] anglesRad);
    public Point3D GetTcpPosition();
}
```

**Acceptance Criteria:**
- [ ] Load STL files cho 6 links + base
- [ ] Parse DH parameters từ config
- [ ] Build transform hierarchy đúng
- [ ] Model hiển thị đúng vị trí home

**Deliverables:**
- RobotModelService.cs
- RobotModel.cs
- RobotLink.cs
- STL loader

---

#### P1-14: Robot 3D Visualization

| Attribute | Value |
|-----------|-------|
| **Priority** | P1 - High |
| **Dependencies** | P1-12, P1-13 |
| **Estimated Effort** | Medium |

**Description:**
Hiển thị robot model trong 3D viewport với khả năng update joint angles.

**Features:**
- Hiển thị robot với vật liệu phù hợp
- Update transforms khi joint angles thay đổi
- Forward Kinematics để tính transform mỗi link
- TCP marker

**Transform Chain (DH Convention):**

```
T_0_1 = Rz(θ1) * Tz(d1) * Tx(a1) * Rx(α1)
T_0_2 = T_0_1 * T_1_2
...
T_0_6 = T_0_1 * T_1_2 * T_2_3 * T_3_4 * T_4_5 * T_5_6
T_0_TCP = T_0_6 * T_TCP_offset
```

**Acceptance Criteria:**
- [ ] Robot hiển thị đúng trong viewport
- [ ] Thay đổi joint angle → Robot update position
- [ ] FK tính đúng TCP position
- [ ] Material/color phù hợp

**Deliverables:**
- RobotVisualizer.cs
- FK implementation
- Joint angle binding

---

#### P1-15: Integration Testing

| Attribute | Value |
|-----------|-------|
| **Priority** | P2 - Medium |
| **Dependencies** | P1-01 through P1-14 |
| **Estimated Effort** | Medium |

**Description:**
Test tích hợp toàn bộ Phase 1 components.

**Test Cases:**

| ID | Test Case | Expected Result |
|----|-----------|-----------------|
| IT-01 | Start C++ Core | Core starts, logs "Ready", binds ports |
| IT-02 | Start C# UI | UI starts, shows MainWindow |
| IT-03 | UI connects to Core | Connection success, status "Connected" |
| IT-04 | PING-PONG | UI sends PING, receives PONG |
| IT-05 | Load Robot Config | Config loaded, no errors |
| IT-06 | Display Robot | Robot visible in viewport |
| IT-07 | Receive Status | UI receives periodic STATUS from Core |
| IT-08 | Update Joint Display | STATUS contains joints → Robot updates |

**Acceptance Criteria:**
- [ ] Tất cả test cases pass
- [ ] Không memory leaks
- [ ] Graceful shutdown cả UI và Core
- [ ] Error handling hoạt động

**Deliverables:**
- Integration test scripts
- Test report

---

## 4. TECH STACK SUMMARY

### 4.1. C++ Core

| Component | Library | Version |
|-----------|---------|---------|
| Build | CMake | 3.20+ |
| Compiler | MSVC | 2022 |
| JSON | nlohmann/json | 3.11+ |
| YAML | yaml-cpp | 0.8+ |
| IPC | cppzmq | 4.10+ |
| Logging | spdlog | 1.12+ |

### 4.2. C# UI

| Component | Package | Version |
|-----------|---------|---------|
| Framework | .NET | 6.0+ |
| UI | WPF | Built-in |
| 3D | HelixToolkit.Wpf | 2.24+ |
| IPC | NetMQ | 4.0+ |
| MVVM | CommunityToolkit.Mvvm | 8.2+ |
| Logging | Serilog | 3.1+ |
| JSON | System.Text.Json | Built-in |

---

## 5. DEFINITION OF DONE

### Phase 1 is DONE when:

- [ ] **All Tasks Completed**: P1-01 through P1-15 đều Done
- [ ] **Milestone Achieved**: "Hello Robot" criteria met
- [ ] **Code Quality**:
  - [ ] No compiler warnings
  - [ ] Code reviewed
  - [ ] Follows style guide
- [ ] **Documentation**:
  - [ ] README updated
  - [ ] API documented
- [ ] **Testing**:
  - [ ] Unit tests pass
  - [ ] Integration tests pass
- [ ] **No Regressions**: Existing functionality still works

---

## 6. RISKS & MITIGATIONS

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| ZeroMQ version incompatibility | Medium | Low | Pin versions, test early |
| Helix Toolkit performance | Medium | Medium | Benchmark với robot model, optimize mesh |
| STL file format issues | Low | Low | Use standard STL, provide sample files |
| Config format changes | Low | Medium | Version config files |

---

## 7. DEPENDENCIES ON OTHER PHASES

| Phase | Dependency | Type |
|-------|------------|------|
| Phase 2 | Uses IPC, Config, Logger | Required |
| Phase 2 | Uses Robot Model | Required |
| Phase 2 | Extends HMI Shell | Required |
| Phase 3 | All Phase 1 components | Required |
| Phase 4 | All Phase 1 components | Required |

---

## APPENDIX

### A. Sample Code Snippets

#### A.1. C++ Main Entry Point

```cpp
// main.cpp
#include "logging/Logger.hpp"
#include "config/ConfigManager.hpp"
#include "ipc/IpcServer.hpp"

int main(int argc, char* argv[]) {
    // Initialize logging
    robot_controller::Logger::init("logs/core.log", "debug");
    LOG_INFO("Robot Controller Core starting...");

    // Load configuration
    auto& config = robot_controller::ConfigManager::instance();
    if (!config.loadSystemConfig("config/system_config.yaml")) {
        LOG_ERROR("Failed to load system config");
        return 1;
    }
    if (!config.loadRobotConfig("config/robot_config.yaml")) {
        LOG_ERROR("Failed to load robot config");
        return 1;
    }
    LOG_INFO("Configuration loaded successfully");

    // Start IPC server
    robot_controller::IpcServer server(
        "tcp://*:" + std::to_string(config.systemConfig().req_port),
        "tcp://*:" + std::to_string(config.systemConfig().pub_port)
    );

    // Register handlers
    server.registerHandler("PING", [](const json& payload) {
        return json{
            {"core_version", "1.0.0"},
            {"uptime_ms", 0}  // TODO: actual uptime
        };
    });

    server.start();
    LOG_INFO("IPC Server started on ports {} and {}",
             config.systemConfig().req_port,
             config.systemConfig().pub_port);

    // Main loop
    while (server.isRunning()) {
        // Publish status periodically
        json status = {
            {"state", "IDLE"},
            {"joints", {0, 0, 0, 0, 0, 0}}
        };
        server.publish({{"type", "STATUS"}, {"payload", status}});

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    LOG_INFO("Robot Controller Core shutting down");
    return 0;
}
```

#### A.2. C# App Startup

```csharp
// App.xaml.cs
public partial class App : Application
{
    private IHost? _host;

    protected override async void OnStartup(StartupEventArgs e)
    {
        base.OnStartup(e);

        // Setup logging
        Log.Logger = new LoggerConfiguration()
            .MinimumLevel.Debug()
            .WriteTo.Console()
            .WriteTo.File("logs/ui.log", rollingInterval: RollingInterval.Day)
            .CreateLogger();

        Log.Information("Robot Controller UI starting...");

        // Build host with DI
        _host = Host.CreateDefaultBuilder()
            .ConfigureServices((context, services) =>
            {
                services.AddSingleton<IConfigService, ConfigService>();
                services.AddSingleton<IIpcClientService, IpcClientService>();
                services.AddSingleton<IRobotModelService, RobotModelService>();
                services.AddSingleton<MainViewModel>();
                services.AddSingleton<MainWindow>();
            })
            .Build();

        await _host.StartAsync();

        var mainWindow = _host.Services.GetRequiredService<MainWindow>();
        mainWindow.Show();
    }

    protected override async void OnExit(ExitEventArgs e)
    {
        if (_host != null)
        {
            await _host.StopAsync();
            _host.Dispose();
        }
        Log.Information("Robot Controller UI shut down");
        Log.CloseAndFlush();
        base.OnExit(e);
    }
}
```

### B. Checklist

```
Phase 1 Completion Checklist:

[ ] Project structure created
[ ] Git repository initialized
[ ] C++ CMake builds successfully
[ ] C# solution builds successfully
[ ] IPC protocol documented
[ ] IPC server (C++) works
[ ] IPC client (C#) works
[ ] UI connects to Core
[ ] Config system works (both)
[ ] Logging works (both)
[ ] MainWindow displays
[ ] 3D viewport works
[ ] Robot model loads
[ ] Robot displays in 3D
[ ] Integration tests pass
[ ] Documentation updated
[ ] Code reviewed
```

---

*Document Version: 1.0 | Last Updated: 2026-02-01*
