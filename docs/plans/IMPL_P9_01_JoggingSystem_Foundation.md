# IMPL_P9_01: Jogging System - Foundation (Firmware Simulator + Joint Jog)

| Metadata | Value |
|----------|-------|
| Phase | Phase 9: Robot Jogging |
| Plan ID | IMPL_P9_01 |
| Title | Jogging System Foundation |
| Priority | P0 (Core robot control feature) |
| Status | PLANNED |
| Created | 2026-02-07 |
| Design Doc | [2026-02-07-hybrid-robot-jogging-design.md](./2026-02-07-hybrid-robot-jogging-design.md) |

---

## 1. Overview

### 1.1 Muc tieu

Xay dung he thong Jog co ban cho phep nguoi dung dieu khien tung joint cua robot trong viewport. He thong ho tro ca simulation mode (khi chua co hardware) va san sang cho real-time sync (khi co Teensy).

### 1.2 Scope

- Firmware Simulator (mock grblHAL trong C++ Core)
- Jog IPC messages (JOG_START, JOG_STOP, JOG_MOVE, JOG_STEP)
- JogController trong Core (xu ly logic, soft limits)
- Wire JogPanel UI events to IPC
- Viewport sync tu status feedback

### 1.3 Out of Scope

- Cartesian Jog (can IK - IMPL_P9_02)
- Ruckig S-curve trajectory (IMPL_P9_02)
- Hardware integration voi Teensy that (IMPL_P9_03)
- Seam tracking (Phase 10+)

---

## 2. Prerequisites

- [x] Phase 8.01 hoan thanh (Robot Package + STL loading)
- [x] IPC system hoat dong (ZeroMQ REQ-REP, PUB-SUB)
- [x] JogPanel.xaml da co san
- [x] ViewportService co the update joints
- [x] DH parameters trong robot.yaml

---

## 3. Required Reading

| Document | Section | Purpose |
|----------|---------|---------|
| [Design Doc](./2026-02-07-hybrid-robot-jogging-design.md) | All | Architecture overview |
| [Architecture](../01_ARCHITECTURE_OVERVIEW.md) | Section 4, 5 | Data flow, IPC protocol |
| [grblHAL Research](../../ressearch_doc_md/Tối%20ưu%20grblHAL%20cho%20Robot%206-DOF.md) | Section 2 | G-code format, status report |
| [FSM Design](../../ressearch_doc_md/Thiết%20Kế%20FSM%20Robot%20Công%20Nghiệp%20An%20Toàn.md) | Section 3 | Jog state transitions |

---

## 4. Implementation Steps

### Step 1: Create Jog IPC Payloads

**Goal**: Define message structures for jog commands

**Files to create:**
- `src/core/src/ipc/JogPayloads.hpp`
- `src/core/src/ipc/JogPayloads.cpp`

**Files to modify:**
- `src/core/src/ipc/MessageTypes.hpp` - Add jog message types
- `src/core/CMakeLists.txt` - Add new source file

**Content for JogPayloads.hpp:**

```cpp
#pragma once

#include <array>
#include <nlohmann/json.hpp>

namespace robot_controller {
namespace ipc {

// Jog modes
enum class JogMode : int {
    JOINT = 0,      // Jog individual joints
    CARTESIAN = 1   // Jog in Cartesian space (future)
};

// Jog movement type
enum class JogType : int {
    CONTINUOUS = 0,  // Move while button held
    INCREMENTAL = 1  // Move fixed increment
};

// Request to start jog mode
struct JogStartRequest {
    bool enableDeadman = true;  // Require continuous button hold

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(JogStartRequest, enableDeadman)
};

// Response to jog start
struct JogStartResponse {
    bool success;
    std::string error;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(JogStartResponse, success, error)
};

// Continuous jog move command
struct JogMoveRequest {
    int mode;           // JogMode: 0=Joint, 1=Cartesian
    int axis;           // Joint: 0-5, Cartesian: 0=X,1=Y,2=Z,3=Rx,4=Ry,5=Rz
    int direction;      // -1 or +1
    double speedPercent; // 1-100%

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(JogMoveRequest, mode, axis, direction, speedPercent)
};

// Incremental jog step command
struct JogStepRequest {
    int mode;           // JogMode
    int axis;           // Joint or Cartesian axis
    int direction;      // -1 or +1
    double increment;   // Degrees for joint, mm for Cartesian
    double speedPercent;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(JogStepRequest, mode, axis, direction, increment, speedPercent)
};

// Jog move response (for both continuous and step)
struct JogMoveResponse {
    bool success;
    std::string error;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(JogMoveResponse, success, error)
};

// Jog state (published with robot status)
struct JogState {
    bool enabled;           // Jog mode active
    bool isMoving;          // Currently jogging
    int currentMode;        // Current jog mode
    int currentAxis;        // Current axis being jogged
    int currentDirection;   // Current direction
    double currentSpeed;    // Current speed percent

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(JogState, enabled, isMoving,
        currentMode, currentAxis, currentDirection, currentSpeed)
};

// Helper functions
std::string jogModeToString(JogMode mode);
std::string jogTypeToString(JogType type);

} // namespace ipc
} // namespace robot_controller
```

**Add to MessageTypes.hpp:**

```cpp
// Jog Control (0x0100 - 0x01FF)
JOG_START     = 0x0100,  // Enable jog mode
JOG_STOP      = 0x0101,  // Disable jog mode
JOG_MOVE      = 0x0102,  // Continuous jog command
JOG_STEP      = 0x0103,  // Incremental jog command
```

**Validation:**
- [ ] Build Core project successfully
- [ ] No compiler errors

---

### Step 2: Create Firmware Simulator

**Goal**: Mock grblHAL behavior for testing without hardware

**Files to create:**
- `src/core/src/firmware/FirmwareSimulator.hpp`
- `src/core/src/firmware/FirmwareSimulator.cpp`

**Files to modify:**
- `src/core/CMakeLists.txt` - Add new source files

**Content for FirmwareSimulator.hpp:**

```cpp
#pragma once

#include <array>
#include <string>
#include <queue>
#include <mutex>
#include <atomic>

namespace robot_controller {
namespace firmware {

/**
 * @brief Simulates grblHAL firmware for testing without hardware
 *
 * Behavior:
 * - Parses G-code commands (G1 J0:angle J1:angle ... Fspeed)
 * - Interpolates joint positions toward target
 * - Returns grblHAL-format status reports
 * - Responds with "ok" when motion completes
 */
class FirmwareSimulator {
public:
    FirmwareSimulator();
    ~FirmwareSimulator();

    // Process incoming G-code command
    // Returns true if command accepted, false if buffer full
    bool sendCommand(const std::string& gcode);

    // Get next response ("ok", "error:X", or status report)
    // Returns empty string if no response available
    std::string getResponse();

    // Update simulation (call at ~1000Hz for smooth motion)
    void update(double dt_seconds);

    // Get current joint positions (degrees)
    std::array<double, 6> getJointPositions() const;

    // Get current state string (Idle, Run, Jog, Hold, Alarm)
    std::string getStateString() const;

    // Generate status report in grblHAL format
    // Format: <State|MPos:j0,j1,j2,j3,j4,j5|FS:feed,0>
    std::string generateStatusReport() const;

    // Request status report (like sending '?' to grblHAL)
    void requestStatus();

    // Emergency stop - immediate halt
    void emergencyStop();

    // Reset from alarm state
    void reset();

    // Configuration
    void setJointLimits(int joint, double min, double max);
    void setMaxVelocity(int joint, double velocity_deg_per_sec);

private:
    // Parse G-code and update targets
    bool parseGcode(const std::string& gcode);

    // Interpolate toward target
    void interpolateMotion(double dt);

    // Check if at target
    bool isAtTarget() const;

    // State
    enum class State { IDLE, JOG, RUN, HOLD, ALARM };
    std::atomic<State> m_state{State::IDLE};

    // Joint positions and targets
    mutable std::mutex m_mutex;
    std::array<double, 6> m_positions{0, 0, 0, 0, 0, 0};
    std::array<double, 6> m_targets{0, 0, 0, 0, 0, 0};
    std::array<double, 6> m_velocities{0, 0, 0, 0, 0, 0};

    // Limits and max velocities
    std::array<double, 6> m_minLimits{-170, -190, -120, -185, -120, -350};
    std::array<double, 6> m_maxLimits{170, 45, 156, 185, 120, 350};
    std::array<double, 6> m_maxVelocities{156, 156, 176, 343, 384, 721}; // deg/s

    // Motion parameters
    double m_feedRate{1000.0};  // degrees per minute
    bool m_isJogMode{false};

    // Response queue
    std::queue<std::string> m_responses;
    bool m_statusRequested{false};

    // Tolerance for "at target" check
    static constexpr double POSITION_TOLERANCE = 0.001; // degrees
};

} // namespace firmware
} // namespace robot_controller
```

**Key implementation points for .cpp:**

1. `parseGcode()`: Parse "G1 J0:45.5 J1:-30.0 F3600"
   - Extract joint targets from J0-J5 parameters
   - Extract feed rate from F parameter
   - Detect jog commands ($J=...)

2. `interpolateMotion()`: Move toward target at feed rate
   - Calculate velocity for each joint
   - Apply velocity limits
   - Check soft limits before moving

3. `generateStatusReport()`: Return grblHAL format
   - `<Idle|MPos:0.00,0.00,0.00,0.00,0.00,0.00|FS:0,0>`

**Validation:**
- [ ] Unit test: Parse G-code correctly
- [ ] Unit test: Interpolate motion smoothly
- [ ] Unit test: Generate valid status report
- [ ] Unit test: Respect soft limits

---

### Step 3: Create JogController

**Goal**: Core logic for jog operations

**Files to create:**
- `src/core/src/jog/JogController.hpp`
- `src/core/src/jog/JogController.cpp`

**Files to modify:**
- `src/core/CMakeLists.txt` - Add new source files

**Content for JogController.hpp:**

```cpp
#pragma once

#include "../ipc/JogPayloads.hpp"
#include "../firmware/FirmwareSimulator.hpp"
#include "../config/RobotPackageSchema.hpp"
#include <memory>
#include <array>
#include <functional>

namespace robot_controller {
namespace jog {

/**
 * @brief Controls robot jogging operations
 *
 * Responsibilities:
 * - Validate jog requests against current state
 * - Apply soft limits before sending commands
 * - Generate G-code for firmware/simulator
 * - Track jog state for UI feedback
 */
class JogController {
public:
    JogController();
    ~JogController();

    // Initialize with robot parameters
    void initialize(const config::RobotPackage& robotPackage);

    // Set firmware driver (real or simulator)
    void setFirmwareSimulator(std::shared_ptr<firmware::FirmwareSimulator> sim);

    // Enable/disable jog mode
    bool enable();
    bool disable();
    bool isEnabled() const;

    // Continuous jog - call repeatedly while button held
    // Returns error message if failed, empty string if success
    std::string startContinuousJog(int mode, int axis, int direction, double speedPercent);
    std::string stopJog();

    // Incremental jog - single step
    std::string jogStep(int mode, int axis, int direction, double increment, double speedPercent);

    // Get current jog state
    ipc::JogState getState() const;

    // Get current joint positions from firmware
    std::array<double, 6> getJointPositions() const;

    // Update loop (call at control frequency)
    void update(double dt);

    // Safety
    void emergencyStop();
    void setDeadmanActive(bool active);

private:
    // Validate jog parameters
    std::string validateJogRequest(int mode, int axis, int direction, double speedPercent);

    // Check soft limits
    bool checkSoftLimits(int axis, double targetAngle);

    // Generate G-code for jog command
    std::string generateJogGcode(int axis, double targetAngle, double feedRate);

    // Calculate jog increment based on speed and dt
    double calculateJogIncrement(int axis, double speedPercent, double dt);

    // State
    bool m_enabled{false};
    bool m_isJogging{false};
    bool m_deadmanActive{false};
    int m_currentAxis{0};
    int m_currentDirection{0};
    double m_currentSpeed{0.0};

    // Robot parameters
    std::array<double, 6> m_minLimits;
    std::array<double, 6> m_maxLimits;
    std::array<double, 6> m_maxVelocities;

    // Current positions (cached from firmware)
    std::array<double, 6> m_currentPositions{0, 0, 0, 0, 0, 0};

    // Firmware interface
    std::shared_ptr<firmware::FirmwareSimulator> m_firmwareSim;

    // Safety constants
    static constexpr double MIN_SPEED_PERCENT = 1.0;
    static constexpr double MAX_SPEED_PERCENT = 100.0;
    static constexpr double SOFT_LIMIT_MARGIN = 0.5; // degrees from hard limit
};

} // namespace jog
} // namespace robot_controller
```

**Key implementation points for .cpp:**

1. `startContinuousJog()`:
   - Validate state (must be enabled, not in alarm)
   - Validate parameters (axis 0-5, direction -1/+1, speed 1-100%)
   - Calculate target = current + large_increment (for continuous)
   - Check soft limits
   - Generate and send G-code

2. `stopJog()`:
   - Send jog cancel command to firmware
   - Update state

3. `update()`:
   - If continuous jogging, keep sending jog commands
   - Poll firmware status
   - Update current positions

**Validation:**
- [ ] Unit test: Enable/disable jog mode
- [ ] Unit test: Validate parameters correctly
- [ ] Unit test: Soft limit enforcement
- [ ] Unit test: Generate correct G-code

---

### Step 4: Add IPC Handlers for Jog Commands

**Goal**: Wire jog messages to JogController

**Files to modify:**
- `src/core/src/controller/RobotController.hpp` - Add JogController member
- `src/core/src/controller/RobotController.cpp` - Add message handlers

**Changes to RobotController.hpp:**

```cpp
// Add includes
#include "../jog/JogController.hpp"
#include "../firmware/FirmwareSimulator.hpp"

// Add members
private:
    std::unique_ptr<jog::JogController> m_jogController;
    std::shared_ptr<firmware::FirmwareSimulator> m_firmwareSim;
```

**Changes to RobotController.cpp:**

```cpp
// In initialize():
m_firmwareSim = std::make_shared<firmware::FirmwareSimulator>();
m_jogController = std::make_unique<jog::JogController>();
m_jogController->setFirmwareSimulator(m_firmwareSim);

// In registerIpcHandlers():
m_ipcServer->registerHandler(MessageType::JOG_START,
    [this](const std::string& payload) -> std::string {
        auto request = nlohmann::json::parse(payload).get<ipc::JogStartRequest>();

        bool success = m_jogController->enable();

        ipc::JogStartResponse response{success, success ? "" : "Failed to enable jog mode"};
        return nlohmann::json(response).dump();
    });

m_ipcServer->registerHandler(MessageType::JOG_STOP,
    [this](const std::string& payload) -> std::string {
        m_jogController->disable();

        ipc::JogStartResponse response{true, ""};
        return nlohmann::json(response).dump();
    });

m_ipcServer->registerHandler(MessageType::JOG_MOVE,
    [this](const std::string& payload) -> std::string {
        auto request = nlohmann::json::parse(payload).get<ipc::JogMoveRequest>();

        std::string error = m_jogController->startContinuousJog(
            request.mode, request.axis, request.direction, request.speedPercent);

        ipc::JogMoveResponse response{error.empty(), error};
        return nlohmann::json(response).dump();
    });

m_ipcServer->registerHandler(MessageType::JOG_STEP,
    [this](const std::string& payload) -> std::string {
        auto request = nlohmann::json::parse(payload).get<ipc::JogStepRequest>();

        std::string error = m_jogController->jogStep(
            request.mode, request.axis, request.direction,
            request.increment, request.speedPercent);

        ipc::JogMoveResponse response{error.empty(), error};
        return nlohmann::json(response).dump();
    });

// In controlLoop():
// Add firmware simulator update
m_firmwareSim->update(m_cycleTimeMs / 1000.0);
m_jogController->update(m_cycleTimeMs / 1000.0);

// In updateStatus():
// Add joint positions from firmware
auto positions = m_firmwareSim->getJointPositions();
for (int i = 0; i < 6; i++) {
    m_status.jointAngles[i] = positions[i];
}
```

**Validation:**
- [ ] Build Core project successfully
- [ ] Test JOG_START via IPC
- [ ] Test JOG_MOVE via IPC
- [ ] Verify status includes joint positions

---

### Step 5: Add C# IPC Client Methods

**Goal**: Add jog methods to IpcClientService

**Files to modify:**
- `src/ui/RobotController.Common/Messages/JogPayloads.cs` (create)
- `src/ui/RobotController.Common/Messages/MessageTypes.cs`
- `src/ui/RobotController.Common/Services/IIpcClientService.cs`
- `src/ui/RobotController.Common/Services/IpcClientService.cs`

**Create JogPayloads.cs:**

```csharp
using System.Text.Json.Serialization;

namespace RobotController.Common.Messages;

public enum JogMode
{
    Joint = 0,
    Cartesian = 1
}

public enum JogType
{
    Continuous = 0,
    Incremental = 1
}

public class JogStartRequest
{
    [JsonPropertyName("enableDeadman")]
    public bool EnableDeadman { get; set; } = true;
}

public class JogStartResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; set; }

    [JsonPropertyName("error")]
    public string Error { get; set; } = "";
}

public class JogMoveRequest
{
    [JsonPropertyName("mode")]
    public int Mode { get; set; }

    [JsonPropertyName("axis")]
    public int Axis { get; set; }

    [JsonPropertyName("direction")]
    public int Direction { get; set; }

    [JsonPropertyName("speedPercent")]
    public double SpeedPercent { get; set; }
}

public class JogStepRequest
{
    [JsonPropertyName("mode")]
    public int Mode { get; set; }

    [JsonPropertyName("axis")]
    public int Axis { get; set; }

    [JsonPropertyName("direction")]
    public int Direction { get; set; }

    [JsonPropertyName("increment")]
    public double Increment { get; set; }

    [JsonPropertyName("speedPercent")]
    public double SpeedPercent { get; set; }
}

public class JogMoveResponse
{
    [JsonPropertyName("success")]
    public bool Success { get; set; }

    [JsonPropertyName("error")]
    public string Error { get; set; } = "";
}

public class JogState
{
    [JsonPropertyName("enabled")]
    public bool Enabled { get; set; }

    [JsonPropertyName("isMoving")]
    public bool IsMoving { get; set; }

    [JsonPropertyName("currentMode")]
    public int CurrentMode { get; set; }

    [JsonPropertyName("currentAxis")]
    public int CurrentAxis { get; set; }

    [JsonPropertyName("currentDirection")]
    public int CurrentDirection { get; set; }

    [JsonPropertyName("currentSpeed")]
    public double CurrentSpeed { get; set; }
}
```

**Add to MessageTypes.cs:**

```csharp
// Jog Control (0x0100 - 0x01FF)
JOG_START = 0x0100,
JOG_STOP = 0x0101,
JOG_MOVE = 0x0102,
JOG_STEP = 0x0103,
```

**Add to IIpcClientService.cs:**

```csharp
// Jog Control
Task<JogStartResponse> StartJogModeAsync(bool enableDeadman = true);
Task<JogStartResponse> StopJogModeAsync();
Task<JogMoveResponse> JogMoveAsync(JogMode mode, int axis, int direction, double speedPercent);
Task<JogMoveResponse> JogStepAsync(JogMode mode, int axis, int direction, double increment, double speedPercent);
```

**Implement in IpcClientService.cs:**

```csharp
public async Task<JogStartResponse> StartJogModeAsync(bool enableDeadman = true)
{
    var request = new JogStartRequest { EnableDeadman = enableDeadman };
    return await SendRequestAsync<JogStartRequest, JogStartResponse>(
        MessageType.JOG_START, request);
}

public async Task<JogStartResponse> StopJogModeAsync()
{
    return await SendRequestAsync<object, JogStartResponse>(
        MessageType.JOG_STOP, new { });
}

public async Task<JogMoveResponse> JogMoveAsync(JogMode mode, int axis, int direction, double speedPercent)
{
    var request = new JogMoveRequest
    {
        Mode = (int)mode,
        Axis = axis,
        Direction = direction,
        SpeedPercent = speedPercent
    };
    return await SendRequestAsync<JogMoveRequest, JogMoveResponse>(
        MessageType.JOG_MOVE, request);
}

public async Task<JogMoveResponse> JogStepAsync(JogMode mode, int axis, int direction,
    double increment, double speedPercent)
{
    var request = new JogStepRequest
    {
        Mode = (int)mode,
        Axis = axis,
        Direction = direction,
        Increment = increment,
        SpeedPercent = speedPercent
    };
    return await SendRequestAsync<JogStepRequest, JogMoveResponse>(
        MessageType.JOG_STEP, request);
}
```

**Validation:**
- [ ] Build UI project successfully
- [ ] IPC methods compile without errors

---

### Step 6: Wire JogPanel to IPC Commands

**Goal**: Connect UI events to jog IPC calls

**Files to modify:**
- `src/ui/RobotController.UI/Views/Controls/JogPanel.xaml.cs`
- `src/ui/RobotController.UI/ViewModels/MotionControlViewModel.cs`

**Update JogPanel.xaml.cs:**

```csharp
public partial class JogPanel : UserControl
{
    public JogPanel()
    {
        InitializeComponent();
    }

    private async void JogPlus_MouseDown(object sender, MouseButtonEventArgs e)
    {
        if (DataContext is MotionControlViewModel vm && vm.IsJogEnabled)
        {
            Mouse.Capture((UIElement)sender);
            await vm.StartJogAsync(direction: 1);
        }
    }

    private async void JogMinus_MouseDown(object sender, MouseButtonEventArgs e)
    {
        if (DataContext is MotionControlViewModel vm && vm.IsJogEnabled)
        {
            Mouse.Capture((UIElement)sender);
            await vm.StartJogAsync(direction: -1);
        }
    }

    private async void Jog_MouseUp(object sender, MouseButtonEventArgs e)
    {
        Mouse.Capture(null);

        if (DataContext is MotionControlViewModel vm)
        {
            await vm.StopJogAsync();
        }
    }
}
```

**Update MotionControlViewModel.cs:**

```csharp
public partial class MotionControlViewModel : ObservableObject
{
    private readonly IIpcClientService _ipcClient;

    [ObservableProperty]
    private bool _isJogEnabled;

    [ObservableProperty]
    private int _selectedJoint;

    [ObservableProperty]
    private double _jogSpeed = 50.0;

    [ObservableProperty]
    private bool _jogJointMode = true;

    public MotionControlViewModel(IIpcClientService ipcClient)
    {
        _ipcClient = ipcClient;
    }

    [RelayCommand]
    private async Task EnableJogAsync()
    {
        var response = await _ipcClient.StartJogModeAsync();
        IsJogEnabled = response.Success;

        if (!response.Success)
        {
            // Show error
            Debug.WriteLine($"Failed to enable jog: {response.Error}");
        }
    }

    [RelayCommand]
    private async Task DisableJogAsync()
    {
        await _ipcClient.StopJogModeAsync();
        IsJogEnabled = false;
    }

    public async Task StartJogAsync(int direction)
    {
        if (!IsJogEnabled) return;

        var mode = JogJointMode ? JogMode.Joint : JogMode.Cartesian;
        var response = await _ipcClient.JogMoveAsync(mode, SelectedJoint, direction, JogSpeed);

        if (!response.Success)
        {
            Debug.WriteLine($"Jog failed: {response.Error}");
        }
    }

    public async Task StopJogAsync()
    {
        // Send stop by calling JogMove with 0 direction or use JOG_STOP
        await _ipcClient.StopJogModeAsync();

        // Re-enable jog mode for next operation
        if (IsJogEnabled)
        {
            await _ipcClient.StartJogModeAsync();
        }
    }

    [RelayCommand]
    private void SelectJoint(string jointIndex)
    {
        if (int.TryParse(jointIndex, out int index))
        {
            SelectedJoint = index;
        }
    }
}
```

**Validation:**
- [ ] Build UI project successfully
- [ ] Click J1-J6 buttons updates SelectedJoint
- [ ] Jog enable/disable works

---

### Step 7: Update Viewport from Status Feedback

**Goal**: Robot in viewport moves based on firmware status

**Files to modify:**
- `src/ui/RobotController.UI/ViewModels/MainViewModel.cs`
- `src/ui/RobotController.UI/Models/RobotModel3D.cs`
- `src/ui/RobotController.Common/Messages/Payloads.cs` (add joint angles to RobotStatus)

**Add to RobotStatus in Payloads.cs:**

```csharp
public class RobotStatus
{
    // ... existing properties ...

    [JsonPropertyName("jointAngles")]
    public double[] JointAngles { get; set; } = new double[6];
}
```

**Update MainViewModel.cs:**

```csharp
private void OnRobotStatusReceived(RobotStatus status)
{
    // Throttle to 30Hz for UI
    var now = DateTime.Now;
    if ((now - _lastViewportUpdate).TotalMilliseconds < 33) return;
    _lastViewportUpdate = now;

    // Update position displays
    UpdatePositionDisplay(status);

    // Update 3D viewport
    if (_robotModel != null && status.JointAngles != null)
    {
        Application.Current.Dispatcher.Invoke(() =>
        {
            _robotModel.UpdateJointsFromFeedback(status.JointAngles);
        });
    }
}

private DateTime _lastViewportUpdate = DateTime.MinValue;
```

**Update RobotModel3D.cs:**

```csharp
/// <summary>
/// Update joint angles from status feedback and recalculate link transforms
/// </summary>
public void UpdateJointsFromFeedback(double[] jointAngles)
{
    if (jointAngles == null || jointAngles.Length != 6) return;

    for (int i = 0; i < 6; i++)
    {
        _jointAngles[i] = jointAngles[i];
    }

    // Recalculate transforms using DH forward kinematics
    RecalculateLinkTransforms();
}

private void RecalculateLinkTransforms()
{
    // Base is at origin
    var currentTransform = Matrix3D.Identity;

    for (int i = 0; i < _links.Count; i++)
    {
        var link = _links[i];

        if (i == 0)
        {
            // Base link - no joint angle
            link.Transform = new MatrixTransform3D(currentTransform);
        }
        else
        {
            // Apply DH transform for this joint
            var dhParams = _dhParameters[i - 1];
            var jointAngle = _jointAngles[i - 1];

            var dhTransform = CalculateDHTransform(
                dhParams.A,
                dhParams.Alpha,
                dhParams.D,
                dhParams.ThetaOffset + jointAngle);

            currentTransform = dhTransform * currentTransform;
            link.Transform = new MatrixTransform3D(currentTransform);
        }
    }
}

private Matrix3D CalculateDHTransform(double a, double alpha, double d, double theta)
{
    double ct = Math.Cos(theta * Math.PI / 180);
    double st = Math.Sin(theta * Math.PI / 180);
    double ca = Math.Cos(alpha * Math.PI / 180);
    double sa = Math.Sin(alpha * Math.PI / 180);

    return new Matrix3D(
        ct, -st * ca, st * sa, a * ct,
        st, ct * ca, -ct * sa, a * st,
        0, sa, ca, d,
        0, 0, 0, 1
    );
}
```

**Validation:**
- [ ] Build UI project successfully
- [ ] Start Core + UI
- [ ] Enable jog, press +/- on J1
- [ ] Robot in viewport rotates J1
- [ ] Release button, robot stops

---

### Step 8: Integration Testing

**Goal**: End-to-end testing of jog system

**Test scenarios:**

| # | Test | Expected Result |
|---|------|-----------------|
| 1 | Start Core, Start UI | UI connects, status received |
| 2 | Load robot package (KR10) | Robot displays in viewport |
| 3 | Click "Jog Enable" button | Jog mode enabled |
| 4 | Select J1, press + | J1 rotates positive |
| 5 | Release button | J1 stops smoothly |
| 6 | Select J2, press - | J2 rotates negative |
| 7 | Jog J1 to positive limit | Stops at soft limit |
| 8 | Jog J1 to negative limit | Stops at soft limit |
| 9 | Click "Jog Disable" | Jog mode disabled |
| 10 | Press +/- while disabled | Nothing happens |

**Create test script:** `test_jog_e2e.bat`

```batch
@echo off
echo === Jog System E2E Test ===
echo.
echo 1. Start Core in background
start /B cmd /c "cd src\core\build && Release\RobotController.exe"
timeout /t 2

echo 2. Start UI
start /B cmd /c "cd src\ui\RobotController.UI && dotnet run"

echo.
echo Test manually:
echo - Load KUKA KR10 robot
echo - Enable jog mode
echo - Test each joint +/-
echo - Verify viewport updates
echo.
pause
```

**Validation:**
- [ ] All 10 test scenarios pass
- [ ] No crashes or hangs
- [ ] Viewport updates smoothly

---

## 5. Validation Checklist

### 5.1 Build Validation

- [ ] Core project builds without errors
- [ ] Core project builds without warnings (or acceptable warnings)
- [ ] UI project builds without errors
- [ ] All unit tests pass

### 5.2 Functional Validation

- [ ] JOG_START enables jog mode
- [ ] JOG_STOP disables jog mode
- [ ] JOG_MOVE moves selected joint
- [ ] Soft limits prevent over-travel
- [ ] Viewport shows correct joint positions
- [ ] Status updates at ~100Hz
- [ ] UI throttles to ~30Hz

### 5.3 Safety Validation

- [ ] Button release stops jog
- [ ] Window focus loss stops jog
- [ ] Soft limits respected
- [ ] No motion when jog disabled

---

## 6. Files Summary

### New Files

| File | Purpose |
|------|---------|
| `src/core/src/ipc/JogPayloads.hpp` | Jog message structures |
| `src/core/src/ipc/JogPayloads.cpp` | Jog payload helpers |
| `src/core/src/firmware/FirmwareSimulator.hpp` | Simulator header |
| `src/core/src/firmware/FirmwareSimulator.cpp` | Simulator implementation |
| `src/core/src/jog/JogController.hpp` | Jog controller header |
| `src/core/src/jog/JogController.cpp` | Jog controller implementation |
| `src/ui/RobotController.Common/Messages/JogPayloads.cs` | C# jog messages |

### Modified Files

| File | Changes |
|------|---------|
| `src/core/src/ipc/MessageTypes.hpp` | Add jog message types |
| `src/core/CMakeLists.txt` | Add new source files |
| `src/core/src/controller/RobotController.hpp` | Add jog members |
| `src/core/src/controller/RobotController.cpp` | Add IPC handlers |
| `src/ui/RobotController.Common/Messages/MessageTypes.cs` | Add jog types |
| `src/ui/RobotController.Common/Services/IIpcClientService.cs` | Add jog methods |
| `src/ui/RobotController.Common/Services/IpcClientService.cs` | Implement jog methods |
| `src/ui/RobotController.UI/Views/Controls/JogPanel.xaml.cs` | Wire events |
| `src/ui/RobotController.UI/ViewModels/MotionControlViewModel.cs` | Add jog commands |
| `src/ui/RobotController.UI/ViewModels/MainViewModel.cs` | Update viewport |
| `src/ui/RobotController.UI/Models/RobotModel3D.cs` | Add FK update |
| `src/ui/RobotController.Common/Messages/Payloads.cs` | Add joint angles |

---

## 7. Estimated Effort

| Step | Effort | Complexity |
|------|--------|------------|
| Step 1: Jog Payloads | 2 hours | Low |
| Step 2: Firmware Simulator | 4-6 hours | Medium |
| Step 3: JogController | 4-6 hours | Medium |
| Step 4: IPC Handlers | 2 hours | Low |
| Step 5: C# IPC Client | 2 hours | Low |
| Step 6: Wire JogPanel | 2 hours | Low |
| Step 7: Viewport Update | 3-4 hours | Medium |
| Step 8: Integration Test | 2-3 hours | Low |
| **Total** | **21-27 hours** | **3-4 days** |

---

## 8. Next Steps (After P9_01)

1. **IMPL_P9_02**: Ruckig S-curve + Cartesian Jog
   - Integrate Ruckig OTG for smooth motion
   - Implement IK solver
   - Add Cartesian jog mode

2. **IMPL_P9_03**: Hardware Integration
   - USB serial communication with Teensy
   - Switch between Simulator and Real firmware
   - Encoder feedback loop

---

*Plan version: 1.0 | Created: 2026-02-07*
