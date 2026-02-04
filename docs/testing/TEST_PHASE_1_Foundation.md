# TEST PLAN: PHASE 1 - FOUNDATION

| Metadata      | Value                           |
|---------------|---------------------------------|
| Phase         | 1 - Foundation                  |
| Version       | 1.0                             |
| Status        | DRAFT                           |
| Last Updated  | 2026-02-01                      |

---

## 1. Test Overview

### 1.1 Scope

Test plan này cover tất cả components trong Phase 1:
- Project build system (CMake + .NET)
- IPC Layer (ZeroMQ)
- Configuration System
- Logging Framework
- HMI Shell
- Robot Model Loader

### 1.2 Test Levels

| Level | Description | Tools |
|-------|-------------|-------|
| **Unit Test** | Test individual functions/classes | Google Test (C++), xUnit (C#) |
| **Integration Test** | Test component interactions | Custom harness |
| **System Test** | End-to-end Phase 1 workflows | Manual + Automated |

### 1.3 Entry/Exit Criteria

**Entry Criteria:**
- [ ] Source code compiles without errors
- [ ] All dependencies installed
- [ ] Test environment configured

**Exit Criteria:**
- [ ] 100% unit tests pass
- [ ] 100% integration tests pass
- [ ] All milestone checklist items verified
- [ ] No critical/high severity bugs open

---

## 2. Unit Tests

### 2.1 Configuration System Tests

#### TEST-P1-U001: Load Valid YAML Config
| Field | Value |
|-------|-------|
| **ID** | TEST-P1-U001 |
| **Component** | ConfigLoader |
| **Priority** | P0 |
| **Type** | Automated |

**Preconditions:**
- Valid config file exists at `config/robot_config.yaml`

**Test Steps:**
1. Call `ConfigLoader::load("config/robot_config.yaml")`
2. Verify return value is `true`
3. Access config values via `ConfigLoader::get<T>()`

**Test Data:**
```yaml
# test_config.yaml
robot:
  name: "TestRobot"
  dof: 6

joints:
  - name: "J1"
    min_limit: -170.0
    max_limit: 170.0
```

**Expected Results:**
- Load returns `true`
- `config.get<string>("robot.name")` returns `"TestRobot"`
- `config.get<int>("robot.dof")` returns `6`
- `config.get<double>("joints[0].min_limit")` returns `-170.0`

**Test Code:**
```cpp
TEST(ConfigLoaderTest, LoadValidYamlConfig) {
    ConfigLoader config;

    ASSERT_TRUE(config.load("test_data/test_config.yaml"));

    EXPECT_EQ(config.get<std::string>("robot.name"), "TestRobot");
    EXPECT_EQ(config.get<int>("robot.dof"), 6);
    EXPECT_DOUBLE_EQ(config.get<double>("joints[0].min_limit"), -170.0);
}
```

---

#### TEST-P1-U002: Load Invalid Config File
| Field | Value |
|-------|-------|
| **ID** | TEST-P1-U002 |
| **Component** | ConfigLoader |
| **Priority** | P0 |
| **Type** | Automated |

**Preconditions:**
- File does not exist or is malformed

**Test Steps:**
1. Call `ConfigLoader::load("nonexistent.yaml")`
2. Verify return value is `false`
3. Verify error message is set

**Expected Results:**
- Load returns `false`
- `getLastError()` contains meaningful error message

**Test Code:**
```cpp
TEST(ConfigLoaderTest, LoadNonexistentFile) {
    ConfigLoader config;

    EXPECT_FALSE(config.load("nonexistent.yaml"));
    EXPECT_FALSE(config.getLastError().empty());
}

TEST(ConfigLoaderTest, LoadMalformedYaml) {
    // Create malformed yaml file
    std::ofstream f("test_data/malformed.yaml");
    f << "invalid: yaml: content: [";
    f.close();

    ConfigLoader config;
    EXPECT_FALSE(config.load("test_data/malformed.yaml"));
}
```

---

#### TEST-P1-U003: Config Default Values
| Field | Value |
|-------|-------|
| **ID** | TEST-P1-U003 |
| **Component** | ConfigLoader |
| **Priority** | P1 |
| **Type** | Automated |

**Test Steps:**
1. Load config without certain keys
2. Request value with default
3. Verify default is returned

**Test Code:**
```cpp
TEST(ConfigLoaderTest, DefaultValues) {
    ConfigLoader config;
    config.load("test_data/minimal_config.yaml");

    // Key doesn't exist - should return default
    EXPECT_EQ(config.get<int>("nonexistent.key", 42), 42);
    EXPECT_EQ(config.get<std::string>("missing", "default"), "default");
}
```

---

### 2.2 IPC Layer Tests

#### TEST-P1-U010: ZeroMQ Server Start/Stop
| Field | Value |
|-------|-------|
| **ID** | TEST-P1-U010 |
| **Component** | IPCServer |
| **Priority** | P0 |
| **Type** | Automated |

**Test Steps:**
1. Create IPCServer instance
2. Call `start("tcp://*:5555")`
3. Verify server is running
4. Call `stop()`
5. Verify server stopped

**Test Code:**
```cpp
TEST(IPCServerTest, StartStop) {
    IPCServer server;

    ASSERT_TRUE(server.start("tcp://*:5555"));
    EXPECT_TRUE(server.isRunning());

    server.stop();
    EXPECT_FALSE(server.isRunning());
}
```

---

#### TEST-P1-U011: Request-Reply Pattern
| Field | Value |
|-------|-------|
| **ID** | TEST-P1-U011 |
| **Component** | IPCServer, IPCClient |
| **Priority** | P0 |
| **Type** | Automated |

**Test Steps:**
1. Start server on port 5555
2. Register request handler
3. Connect client
4. Send request from client
5. Verify response received

**Test Code:**
```cpp
TEST(IPCTest, RequestReply) {
    IPCServer server;
    server.registerHandler("ping", [](const json& req) {
        return json{{"response", "pong"}};
    });
    server.start("tcp://*:5556");

    IPCClient client;
    client.connect("tcp://localhost:5556");

    auto response = client.request("ping", json{});

    EXPECT_EQ(response["response"], "pong");

    client.disconnect();
    server.stop();
}
```

---

#### TEST-P1-U012: IPC Message Serialization
| Field | Value |
|-------|-------|
| **ID** | TEST-P1-U012 |
| **Component** | IPCMessage |
| **Priority** | P1 |
| **Type** | Automated |

**Test Steps:**
1. Create IPCMessage with various data types
2. Serialize to bytes
3. Deserialize from bytes
4. Verify data integrity

**Test Code:**
```cpp
TEST(IPCMessageTest, Serialization) {
    IPCMessage msg;
    msg.type = MessageType::COMMAND;
    msg.id = 12345;
    msg.payload = json{
        {"command", "jog"},
        {"axis", 0},
        {"velocity", 10.5},
        {"positions", {1.0, 2.0, 3.0, 4.0, 5.0, 6.0}}
    };

    auto bytes = msg.serialize();

    IPCMessage decoded;
    ASSERT_TRUE(decoded.deserialize(bytes));

    EXPECT_EQ(decoded.type, MessageType::COMMAND);
    EXPECT_EQ(decoded.id, 12345);
    EXPECT_EQ(decoded.payload["command"], "jog");
    EXPECT_EQ(decoded.payload["axis"], 0);
    EXPECT_DOUBLE_EQ(decoded.payload["velocity"], 10.5);
}
```

---

#### TEST-P1-U013: IPC Timeout Handling
| Field | Value |
|-------|-------|
| **ID** | TEST-P1-U013 |
| **Component** | IPCClient |
| **Priority** | P1 |
| **Type** | Automated |

**Test Steps:**
1. Connect client to non-existent server
2. Send request with timeout
3. Verify timeout error

**Test Code:**
```cpp
TEST(IPCClientTest, RequestTimeout) {
    IPCClient client;
    client.setRequestTimeout(100);  // 100ms
    client.connect("tcp://localhost:9999");  // No server

    auto start = std::chrono::steady_clock::now();
    auto response = client.request("ping", json{});
    auto elapsed = std::chrono::steady_clock::now() - start;

    EXPECT_TRUE(response.is_null() || response.contains("error"));
    EXPECT_LT(elapsed, std::chrono::milliseconds(200));
}
```

---

### 2.3 Logging Framework Tests

#### TEST-P1-U020: Log Levels
| Field | Value |
|-------|-------|
| **ID** | TEST-P1-U020 |
| **Component** | Logger |
| **Priority** | P1 |
| **Type** | Automated |

**Test Steps:**
1. Configure logger with DEBUG level
2. Log messages at all levels
3. Verify all messages logged
4. Change to ERROR level
5. Verify only ERROR+ messages logged

**Test Code:**
```cpp
TEST(LoggerTest, LogLevels) {
    auto sink = std::make_shared<TestSink>();
    Logger::init(sink, LogLevel::DEBUG);

    LOG_DEBUG("debug message");
    LOG_INFO("info message");
    LOG_WARN("warn message");
    LOG_ERROR("error message");

    EXPECT_EQ(sink->messages.size(), 4);

    sink->clear();
    Logger::setLevel(LogLevel::ERROR);

    LOG_DEBUG("debug");
    LOG_INFO("info");
    LOG_ERROR("error");

    EXPECT_EQ(sink->messages.size(), 1);
    EXPECT_TRUE(sink->messages[0].find("error") != std::string::npos);
}
```

---

#### TEST-P1-U021: Log File Output
| Field | Value |
|-------|-------|
| **ID** | TEST-P1-U021 |
| **Component** | Logger |
| **Priority** | P1 |
| **Type** | Automated |

**Test Steps:**
1. Configure logger with file output
2. Log several messages
3. Flush logger
4. Read log file
5. Verify messages present

**Test Code:**
```cpp
TEST(LoggerTest, FileOutput) {
    const char* logfile = "test_logs/test.log";
    std::filesystem::remove(logfile);

    Logger::initFile(logfile, LogLevel::INFO);

    LOG_INFO("Test message 1");
    LOG_INFO("Test message 2");

    Logger::flush();

    std::ifstream f(logfile);
    std::string content((std::istreambuf_iterator<char>(f)),
                        std::istreambuf_iterator<char>());

    EXPECT_TRUE(content.find("Test message 1") != std::string::npos);
    EXPECT_TRUE(content.find("Test message 2") != std::string::npos);
}
```

---

### 2.4 Robot Model Tests

#### TEST-P1-U030: Load Robot URDF
| Field | Value |
|-------|-------|
| **ID** | TEST-P1-U030 |
| **Component** | RobotModelLoader |
| **Priority** | P0 |
| **Type** | Automated |

**Test Steps:**
1. Load valid URDF file
2. Verify 6 joints loaded
3. Verify joint limits correct
4. Verify link names correct

**Test Code:**
```cpp
TEST(RobotModelTest, LoadURDF) {
    RobotModelLoader loader;

    ASSERT_TRUE(loader.load("models/robot_arm.urdf"));

    auto model = loader.getModel();

    EXPECT_EQ(model.getJointCount(), 6);
    EXPECT_EQ(model.getLinkCount(), 7);  // 6 joints + base

    // Check joint limits
    auto j1 = model.getJoint(0);
    EXPECT_NEAR(j1.min_limit, -170.0 * DEG2RAD, 0.01);
    EXPECT_NEAR(j1.max_limit, 170.0 * DEG2RAD, 0.01);
}
```

---

#### TEST-P1-U031: Robot Model to 3D Mesh
| Field | Value |
|-------|-------|
| **ID** | TEST-P1-U031 |
| **Component** | RobotModelLoader |
| **Priority** | P1 |
| **Type** | Automated |

**Test Steps:**
1. Load robot model
2. Get mesh data for each link
3. Verify mesh has vertices and faces

**Test Code:**
```cpp
TEST(RobotModelTest, GetMeshData) {
    RobotModelLoader loader;
    loader.load("models/robot_arm.urdf");

    auto model = loader.getModel();

    for (int i = 0; i < model.getLinkCount(); i++) {
        auto mesh = model.getLinkMesh(i);

        EXPECT_GT(mesh.vertices.size(), 0);
        EXPECT_GT(mesh.faces.size(), 0);
        EXPECT_EQ(mesh.vertices.size() % 3, 0);  // x, y, z
    }
}
```

---

## 3. Integration Tests

### 3.1 IPC Integration

#### TEST-P1-I001: UI-Core Communication
| Field | Value |
|-------|-------|
| **ID** | TEST-P1-I001 |
| **Component** | UI (C#) ↔ Core (C++) |
| **Priority** | P0 |
| **Type** | Automated |

**Preconditions:**
- Core executable built
- UI application built

**Test Steps:**
1. Start Core process
2. Wait for IPC server ready (port 5555)
3. Start UI application
4. UI connects to Core
5. UI sends "get_status" request
6. Core responds with status
7. Verify response in UI

**Test Procedure:**
```powershell
# Terminal 1: Start Core
.\build\core\RobotCore.exe --config config/test_config.yaml

# Terminal 2: Run integration test
.\build\tests\integration_test.exe --test=UI_Core_Communication
```

**Expected Results:**
- Core starts and logs "IPC Server started on tcp://*:5555"
- UI connects within 2 seconds
- Status response received with valid data
- No errors in either log

**Pass Criteria:**
- [ ] Connection established < 2s
- [ ] Response received < 100ms
- [ ] Response contains required fields (version, state, timestamp)

---

#### TEST-P1-I002: IPC High-Frequency Messages
| Field | Value |
|-------|-------|
| **ID** | TEST-P1-I002 |
| **Component** | IPCServer, IPCClient |
| **Priority** | P1 |
| **Type** | Automated + Performance |

**Test Steps:**
1. Start Core with IPC server
2. Connect client
3. Send 1000 requests as fast as possible
4. Measure throughput and latency

**Test Code:**
```cpp
TEST(IPCIntegration, HighFrequency) {
    // Assume server already running
    IPCClient client;
    client.connect("tcp://localhost:5555");

    const int NUM_REQUESTS = 1000;
    std::vector<double> latencies;

    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < NUM_REQUESTS; i++) {
        auto req_start = std::chrono::high_resolution_clock::now();

        auto response = client.request("ping", json{{"seq", i}});

        auto req_end = std::chrono::high_resolution_clock::now();
        auto latency = std::chrono::duration<double, std::milli>(
            req_end - req_start).count();
        latencies.push_back(latency);

        ASSERT_FALSE(response.is_null());
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto total_time = std::chrono::duration<double>(end - start).count();

    double avg_latency = std::accumulate(latencies.begin(), latencies.end(), 0.0)
                        / latencies.size();
    double max_latency = *std::max_element(latencies.begin(), latencies.end());
    double throughput = NUM_REQUESTS / total_time;

    std::cout << "Throughput: " << throughput << " req/s" << std::endl;
    std::cout << "Avg latency: " << avg_latency << " ms" << std::endl;
    std::cout << "Max latency: " << max_latency << " ms" << std::endl;

    // Performance requirements
    EXPECT_GT(throughput, 500);      // > 500 req/s
    EXPECT_LT(avg_latency, 5.0);     // < 5ms average
    EXPECT_LT(max_latency, 50.0);    // < 50ms max
}
```

**Pass Criteria:**
- [ ] Throughput > 500 requests/second
- [ ] Average latency < 5ms
- [ ] Max latency < 50ms
- [ ] Zero message loss

---

### 3.2 Config-Logger Integration

#### TEST-P1-I010: Config-Driven Logging
| Field | Value |
|-------|-------|
| **ID** | TEST-P1-I010 |
| **Component** | ConfigLoader + Logger |
| **Priority** | P1 |
| **Type** | Automated |

**Test Steps:**
1. Create config with logging settings
2. Load config
3. Initialize logger from config
4. Verify logging behavior matches config

**Test Config:**
```yaml
logging:
  level: DEBUG
  file: "logs/robot.log"
  console: true
  max_file_size_mb: 10
  max_files: 5
```

**Test Code:**
```cpp
TEST(ConfigLoggerIntegration, ConfigDrivenLogging) {
    ConfigLoader config;
    config.load("test_data/logging_config.yaml");

    Logger::initFromConfig(config);

    EXPECT_EQ(Logger::getLevel(), LogLevel::DEBUG);
    EXPECT_TRUE(Logger::isFileEnabled());
    EXPECT_TRUE(Logger::isConsoleEnabled());

    LOG_DEBUG("Test debug message");
    Logger::flush();

    // Verify file was created
    EXPECT_TRUE(std::filesystem::exists("logs/robot.log"));
}
```

---

### 3.3 HMI Integration

#### TEST-P1-I020: HMI Startup Sequence
| Field | Value |
|-------|-------|
| **ID** | TEST-P1-I020 |
| **Component** | HMI Shell |
| **Priority** | P0 |
| **Type** | Manual + Automated |

**Preconditions:**
- Core process running
- Config files present

**Test Steps:**
1. Launch HMI application
2. Observe startup sequence
3. Verify main window appears
4. Verify 3D viewport loads
5. Verify status bar shows "Connected"

**Expected Results:**
| Step | Expected | Timeout |
|------|----------|---------|
| Window appears | Main window visible | 5s |
| 3D viewport | Robot model visible | 10s |
| Status bar | Shows "Connected to Core" | 5s |
| No errors | No error dialogs | - |

**Manual Verification Checklist:**
- [ ] Main window appears without errors
- [ ] Window title shows application name and version
- [ ] 3D viewport shows robot model
- [ ] Robot model has correct number of links (7)
- [ ] Status bar shows connection status
- [ ] Menu bar is functional
- [ ] No console errors in Output window

---

#### TEST-P1-I021: 3D Viewport Robot Display
| Field | Value |
|-------|-------|
| **ID** | TEST-P1-I021 |
| **Component** | 3D Viewport (Helix Toolkit) |
| **Priority** | P0 |
| **Type** | Manual |

**Test Steps:**
1. Start HMI with robot model loaded
2. Verify robot displayed in viewport
3. Test camera controls (rotate, pan, zoom)
4. Verify joint positions update when changed

**Manual Verification Checklist:**
- [ ] Robot model visible in center of viewport
- [ ] All 6 links rendered correctly
- [ ] Left-click drag: Rotate camera
- [ ] Right-click drag: Pan camera
- [ ] Mouse wheel: Zoom in/out
- [ ] Home button resets camera view
- [ ] Coordinate system axes visible (X=Red, Y=Green, Z=Blue)

---

## 4. System Tests

### 4.1 Milestone Verification

#### TEST-P1-S001: Milestone "Hello Robot"
| Field | Value |
|-------|-------|
| **ID** | TEST-P1-S001 |
| **Component** | All Phase 1 |
| **Priority** | P0 |
| **Type** | Manual System Test |

**Objective:** Verify all Phase 1 milestone criteria are met.

**Test Procedure:**

**Step 1: Build Verification**
```powershell
# Build all components
cmake --build build --config Release

# Expected: Build completes with 0 errors
```
- [ ] PASS: Build completes successfully
- [ ] Core executable created: `build/core/RobotCore.exe`
- [ ] UI executable created: `build/ui/RobotHMI.exe`

**Step 2: Core Startup**
```powershell
# Start Core process
.\build\core\RobotCore.exe --config config/robot_config.yaml
```
- [ ] PASS: Core starts without errors
- [ ] PASS: Log shows "IPC Server started"
- [ ] PASS: Log shows "Config loaded successfully"

**Step 3: UI Startup**
```powershell
# Start UI application
.\build\ui\RobotHMI.exe
```
- [ ] PASS: UI window appears within 5 seconds
- [ ] PASS: 3D viewport shows robot model
- [ ] PASS: Status bar shows "Connected"

**Step 4: IPC Verification**
- [ ] PASS: UI can request status from Core
- [ ] PASS: Core responds with valid status JSON
- [ ] PASS: Round-trip time < 100ms

**Step 5: Config Verification**
- [ ] PASS: Robot name loaded from config
- [ ] PASS: Joint limits loaded from config
- [ ] PASS: IPC port loaded from config

**Step 6: Logging Verification**
- [ ] PASS: Log file created in logs/ directory
- [ ] PASS: Console output visible (if configured)
- [ ] PASS: Log entries have timestamp, level, message

**Overall Result:**
- [ ] **MILESTONE PASSED**: All criteria met
- [ ] **MILESTONE FAILED**: List failed items

---

## 5. Test Environment

### 5.1 Hardware Requirements

| Component | Specification |
|-----------|---------------|
| CPU | Intel i5 or equivalent |
| RAM | 8 GB minimum |
| GPU | DirectX 11 compatible |
| Storage | 10 GB free space |
| OS | Windows 10/11 64-bit |

### 5.2 Software Requirements

| Software | Version | Purpose |
|----------|---------|---------|
| Visual Studio | 2022 | Build |
| CMake | 3.20+ | Build system |
| .NET SDK | 8.0+ | UI build |
| Google Test | Latest | C++ unit tests |
| xUnit | Latest | C# unit tests |

### 5.3 Test Data

| File | Location | Purpose |
|------|----------|---------|
| test_config.yaml | test_data/ | Valid config |
| malformed.yaml | test_data/ | Invalid config |
| robot_arm.urdf | models/ | Robot model |
| logging_config.yaml | test_data/ | Logging settings |

---

## 6. Test Execution

### 6.1 Automated Test Execution

```powershell
# Run all C++ unit tests
.\build\tests\unit_tests.exe --gtest_output=xml:test_results/unit_tests.xml

# Run all C# unit tests
dotnet test src/ui/RobotHMI.Tests --logger "trx;LogFileName=test_results/ui_tests.trx"

# Run integration tests
.\build\tests\integration_tests.exe --gtest_output=xml:test_results/integration.xml
```

### 6.2 Test Report Template

```
================================================================================
                         PHASE 1 TEST REPORT
================================================================================

Date: _______________
Tester: _______________
Build Version: _______________

SUMMARY
-------
Unit Tests:        ___ / ___ passed
Integration Tests: ___ / ___ passed
System Tests:      ___ / ___ passed

UNIT TEST RESULTS
-----------------
[x] TEST-P1-U001: Load Valid YAML Config          PASS
[x] TEST-P1-U002: Load Invalid Config File        PASS
[ ] TEST-P1-U003: Config Default Values           FAIL - See notes
...

FAILED TESTS
------------
TEST-P1-U003: Config Default Values
  - Expected: 42
  - Actual: 0
  - Notes: Default value not returned when key missing

INTEGRATION TEST RESULTS
------------------------
[x] TEST-P1-I001: UI-Core Communication           PASS
[x] TEST-P1-I002: IPC High-Frequency Messages     PASS
  - Throughput: 850 req/s
  - Avg latency: 1.2ms
  - Max latency: 15ms
...

SYSTEM TEST RESULTS
-------------------
[x] TEST-P1-S001: Milestone "Hello Robot"         PASS
  - All checklist items verified

ISSUES FOUND
------------
1. [BUG-001] Config default value not working
   - Severity: Medium
   - Status: Open

RECOMMENDATION
--------------
[ ] PHASE 1 APPROVED - Ready for Phase 2
[ ] PHASE 1 NOT APPROVED - Issues must be fixed

Signed: _______________
Date: _______________
================================================================================
```

---

## 7. Traceability Matrix

| Requirement | Test Case(s) | Status |
|-------------|--------------|--------|
| Config loads from YAML | TEST-P1-U001, U002, U003 | - |
| IPC communication works | TEST-P1-U010-U013, I001, I002 | - |
| Logging to file/console | TEST-P1-U020, U021, I010 | - |
| Robot model loads | TEST-P1-U030, U031 | - |
| HMI displays robot | TEST-P1-I020, I021 | - |
| Milestone "Hello Robot" | TEST-P1-S001 | - |

---

## 8. Document History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2026-02-01 | System | Initial version |
