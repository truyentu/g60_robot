# FEATURE LIST: 6-DOF Robot Controller

**Project:** Commercial 6-DOF Robot Controller for MIG/MAG Welding
**Architecture:** C++ Core + C# WPF HMI + Teensy Firmware
**Version:** 1.0
**Created:** 2026-02-03

---

## 1. CORE PLATFORM FEATURES

### 1.1 Three-Layer Architecture
- **UI Layer (C# WPF .NET 8)**
  - HMI Shell Window
  - 3D Visualization (Helix Toolkit SharpDX)
  - Program Editor
  - Config Panel
  - Welding Scope
- **Core Logic Layer (C++ 17/20)**
  - State Manager
  - Motion Planner
  - Welding Sequencer
  - Vision Pipeline
  - Kinematics (IK/FK)
  - Ruckig OTG
  - Safety Interlock
- **Firmware Layer (Teensy 4.1 + grblHAL)**
  - Real-time G-code Execution
  - Step Generator (600MHz)
  - Encoder Feedback
  - Safety I/O
  - Weld I/O

### 1.2 IPC Communication (ZeroMQ)
- REQ-REP Pattern (Commands)
  - JOG_START, JOG_STOP
  - MOVE_TO
  - PROGRAM_RUN, PROGRAM_STOP
  - SET_CONFIG, GET_CONFIG
- PUB-SUB Pattern (State Broadcasting)
  - ROBOT_STATE
  - ALARM_STATE
  - WELDING_STATE
  - IO_STATE
- PUSH-PULL Pattern (Logs/Events)
- Binary Message Format với CRC32 Checksum
- Heartbeat Monitoring

### 1.3 Configuration System
- **Robot Config (YAML)**
  - DH Parameters (Modified DH Convention)
  - Joint Limits (min, max, velocity, acceleration)
  - TCP Offset
- **System Config (YAML)**
  - IPC Ports
  - Logging Settings
  - Control Cycle Time
- **UI Config (JSON)**
  - Theme (Dark/Light)
  - Language
  - Viewport Settings
  - Connection Settings

### 1.4 Logging Framework
- **C++ (spdlog)**
  - Console logging with colors
  - File logging with rotation
  - Configurable log levels
  - Thread-safe
- **C# (Serilog)**
  - Console sink
  - File sink with daily rotation
  - Structured logging

---

## 2. MOTION CONTROL FEATURES

### 2.1 State Machine (ISO 10218-1)
- **States**
  - BOOT (Initialization)
  - ERROR_LOCKOUT
  - ESTOP_ACTIVE (Category 0 Stop)
  - ESTOP_RESET_NEEDED
  - IDLE (Servos Off, Brakes Engaged)
  - ARMING (Enable Drives, Release Brakes)
  - OPERATIONAL
    - MANUAL_IDLE
    - MANUAL_JOGGING
    - AUTO_IDLE
    - AUTO_RUNNING
    - AUTO_PAUSED
  - STOPPING (Cat 1/Cat 2)
- **Operating Modes**
  - T1 (Manual Reduced Speed - 250mm/s max)
  - T2 (Manual High Speed)
  - AUTO (Automatic Mode)
- **State Transitions**
  - Guard Conditions (Interlocks, Deadman)
  - Event-driven Transitions
  - Logging for all transitions

### 2.2 Safety System
- **Safety Signals (Dual-Channel)**
  - E-Stop (NC Contact)
  - Safety Door/Light Curtain
  - Deadman Switch (3-position)
- **Soft Limits**
  - Positive/Negative Limits per Axis
- **Discrepancy Detection**
  - 50ms Timeout for Channel Mismatch
- **Stop Categories (IEC 60204-1)**
  - Category 0: Immediate Power Removal
  - Category 1: Controlled Stop then Power Off
  - Category 2: Controlled Stop, Power Maintained
- **Safety Layers**
  - Layer 1: Hardware Safety (E-Stop Circuit, Safety Relay, Motor Brake)
  - Layer 2: Firmware Safety (Hard Limits, Watchdog Timer)
  - Layer 3: Core Safety (Heartbeat, Soft Limits, Velocity Monitor)
  - Layer 4: UI Safety (Two-Handed Operation, Enable Button)

### 2.3 Kinematics
- **Forward Kinematics (FK)**
  - DH Transform Computation
  - Transform Chain (Base → Link1 → ... → Link6 → TCP)
  - Jacobian Computation
  - Euler Angle Extraction (ZYX)
- **Inverse Kinematics (IK)**
  - Analytical Solution for 6-DOF
  - 8 Configuration Solutions
    - Shoulder Left/Right
    - Elbow Up/Down
    - Wrist Flip/No-Flip
  - Closest Solution Selection
  - Joint Limit Validation
  - Singularity Detection

### 2.4 Trajectory Generation (Ruckig OTG)
- **Motion Profiles**
  - Jerk-Limited S-Curve
  - Time-Optimal Trajectories
  - Online Trajectory Generation
- **Motion Types**
  - Point-to-Point (PTP)
  - Linear (MOVL)
  - Circular (MOVC)
- **Jog Mode**
  - Velocity-based Jogging
  - Smooth Acceleration/Deceleration
  - Emergency Stop
- **Phase Synchronization**
  - Multi-axis Coordination
  - Synchronized Motion

### 2.5 Motion Controller
- **Motion Loop (1kHz)**
  - Read Joint Positions
  - Update Trajectory
  - Send Commands to Hardware
- **Jog Controller**
  - Joint Jog (6 Axes)
  - Cartesian Jog (X, Y, Z, Rx, Ry, Rz)
  - Tool Jog (Tool Coordinate System)
  - Speed Ratio Control (0-100%)
  - T1 Mode Speed Limit (250mm/s)
- **Position Control**
  - Move to Joint Position
  - Move to Cartesian Position
  - Velocity/Acceleration Limits

### 2.6 Firmware Communication (grblHAL)
- **Serial Protocol**
  - USB CDC @ 115200 baud
  - Character Counting Flow Control
  - 4096-byte RX Buffer
- **G-code Commands**
  - G1 (Linear Move with Joint Angles)
  - Real-time Commands (Feed Hold, Jog Cancel)
  - Settings Commands ($xxx)
- **Status Reporting**
  - Machine Position (6 joints)
  - Feed Rate, Spindle Speed
  - Input Pin States
  - Override Values
- **Hardware Interface Abstraction**
  - IHardwareInterface (Abstract)
  - GrblHalDriver (Real Hardware)
  - VirtualController (Simulation)

### 2.7 Simulation Mode
- Simulated Joint Motion
- Simulated Servo Enable/Disable
- Simulated I/O
- Fault Injection for Testing
- Thread-safe Access

---

## 3. HMI FEATURES

### 3.1 Main Window Shell
- **Layout**
  - Menu Bar
  - Toolbar
  - Navigation Panel (Manual, Program, Config, Diagnostics)
  - Main Content Area
  - Status Bar
- **Responsive Design**
- **Dark/Light Theme**

### 3.2 3D Viewport (Helix Toolkit)
- **Camera Controls**
  - Orbit (Mouse Drag)
  - Pan (Middle Mouse)
  - Zoom (Scroll)
- **Visual Elements**
  - Grid Plane
  - Coordinate Axes (X-Red, Y-Green, Z-Blue)
  - Robot Model (STL)
  - TCP Marker
  - Ghost Robot (Transparent Target Position)
- **Performance**
  - 30+ FPS
  - DirectX Rendering

### 3.3 Robot Model Loader
- Load STL Files (7 Links)
- Parse DH Parameters from Config
- Build Transform Hierarchy
- Material/Color Configuration

### 3.4 Jog Panel
- **Mode Selection**
  - Joint Mode
  - World Mode (Cartesian)
  - Tool Mode
- **Jog Buttons**
  - +/- for Each Axis
  - Hold-to-Jog Behavior
- **Speed Slider (1-100%)**
- **Keyboard Shortcuts**

### 3.5 Status Display
- **State Indicator (Color-Coded)**
- **Mode Indicator (T1/T2/AUTO)**
- **Joint Positions (6 values)**
- **TCP Position (X, Y, Z, Rx, Ry, Rz)**
- **Velocity Display**
- **Error Messages**
- **Real-time Updates (10Hz+)**

### 3.6 Position Display
- Current Joint Angles
- Current Cartesian Pose
- Target Position
- Following Error

### 3.7 Motion Control Panel
- Servo On/Off
- Mode Selection
- Speed Override
- Start/Stop/Pause Controls

---

## 4. WELDING FEATURES

### 4.1 Welding Sequencer (FSM)
- **Welding States**
  - IDLE
  - PREFLOW (Gas Pre-flow)
  - IGNITION (Arc Start)
  - STABILIZE (Wait Arc OK)
  - WELD (Welding)
  - CRATER (Crater Fill)
  - BURNBACK (Wire Burnback)
  - POSTFLOW (Gas Post-flow)
  - FAULT
- **Timing Parameters**
  - PreFlowTime (100-2000ms)
  - IgnitionTimeout (500-5000ms)
  - StabilizeTime (50-1000ms)
  - CraterTime (0-2000ms)
  - BurnbackTime (0-500ms)
  - PostFlowTime (200-5000ms)
- **Commands**
  - Start Weld
  - Stop Weld (Graceful)
  - Emergency Stop
  - Reset

### 4.2 I/O Interface
- **Digital Outputs**
  - DO_GAS_VALVE (Gas Solenoid)
  - DO_ARC_START (Arc Start Command)
  - DO_WIRE_INCH (Wire Jog)
  - DO_WIRE_RETRACT (Wire Retract)
  - DO_TORCH_COLLISION (Reset)
- **Digital Inputs**
  - DI_ARC_OK (Arc Established)
  - DI_READY (Power Source Ready)
  - DI_WELD_ERROR (Error Signal)
  - DI_WIRE_STUCK (Wire Stuck)
  - DI_GAS_PRESSURE_OK (Gas OK)
- **Analog Outputs (0-10V)**
  - AO_WFS_REF (Wire Feed Speed)
  - AO_VOLT_REF (Voltage Reference)
- **Signal Debouncing**
  - Asymmetric Debounce for ARC_OK (20ms rise, 50ms fall)
  - Configurable Debounce Times

### 4.3 Fault Detection & Recovery
- **Fault Codes**
  - IGNITION_FAILURE
  - ARC_LOST
  - WIRE_STUCK
  - GAS_PRESSURE_LOW
  - POWER_SOURCE_ERROR
  - COMMUNICATION_ERROR
  - EMERGENCY_STOP
- **Auto-Retry for Ignition**
- **Recovery Procedures**
- **Detailed Logging**

### 4.4 Arc Monitor
- Arc On Time
- Arc Off Time
- Arc Stability (%)
- Arc Lost Count
- Average WFS
- Average Voltage
- Alarm on Low Stability

### 4.5 Weaving Patterns
- **Pattern Types**
  - Sine Wave
  - Triangle Wave
  - Trapezoid Wave
  - Circle Pattern
  - Figure-8 (Lissajous)
  - L-Type Pattern
- **Weaving Parameters**
  - Amplitude (mm)
  - Frequency (Hz)
  - Dwell Time Left (ms)
  - Dwell Time Right (ms)
  - Phase Offset
- **Weaving Frame Calculation**
  - Tangent (Weld Direction)
  - Binormal (Weave Direction)
  - Normal
- **Integration with Trajectory**
  - Smooth Enable/Disable
  - Ramp Up/Down

### 4.6 Weld Program Manager
- **Instruction Types**
  - MOVJ (Joint Move)
  - MOVL (Linear Move)
  - WELD_START
  - WELD_END
  - WELD_LINE (Move + Weld)
  - WELD_ARC (Arc Move + Weld)
  - SET_WFS, SET_VOLTAGE
  - SET_WEAVING
  - WAIT, COMMENT
- **Program Structure**
  - Named Positions
  - Default Config
  - Instruction List
- **File Operations**
  - Load/Save JSON
  - Create/Edit/Delete
- **Execution Control**
  - Start/Pause/Resume/Stop
  - Step-by-Step Mode
- **Teaching Mode**
  - Record Positions
  - Insert/Delete Instructions

### 4.7 Welding HMI
- **Welding Control Panel**
  - State Display
  - Arc/Gas Indicators
  - Parameter Sliders (WFS, Voltage, Travel Speed)
  - Weaving Config
  - Start/Stop/Reset Buttons
  - Wire Inch Control
- **Weld Monitor View**
  - Real-time Current/Voltage Bars
  - Waveform Display
  - Statistics (Arc Stability, Wire Used, Faults)
  - Data Logging
  - Recording Function
- **Weave Preview Panel**
  - Pattern Visualization
  - Real-time Preview

---

## 5. VISION FEATURES

### 5.1 Sensor Drivers
- **GigE Vision Client**
  - Device Discovery
  - UDP Streaming
  - Jumbo Frames (MTU 9000)
  - GenICam Feature Access
- **Laser Profiler Driver (Hikrobot MVS)**
  - Initialize/Open Device
  - Start/Stop Acquisition
  - Trigger Modes (Free Run, Software, Hardware)
  - Encoder Divider Configuration
  - Profile Data (X, Z, Intensity, Timestamp, Encoder Count)
- **Encoder Synchronization**
  - Hardware Trigger from Encoder
  - Pose Buffer with Encoder Count
  - Exact Position Lookup
- **Lock-Free Ring Buffer**
  - No Memory Allocation
  - Overflow Handling
  - 100k+ profiles/sec

### 5.2 Point Cloud Processing
- **Noise Filtering**
  - Statistical Outlier Removal (SOR)
  - Radius Outlier Removal (ROR)
  - Intensity-based Filter
  - Combined Pipeline
- **Coordinate Transformation**
  - Sensor → Robot Base
  - Base → Sensor
  - Sensor → Tool
  - Point Cloud Transform
- **V-Groove Detector (RANSAC)**
  - Profile Split (Left/Right)
  - Line Fitting per Half
  - Intersection Calculation
  - Groove Parameters (Angle, Width, Depth)
  - Confidence Score
- **Fillet Detector (Plane Intersection)**
  - Dominant Plane Fitting
  - Secondary Plane Fitting
  - Intersection Line Calculation
  - Seam Direction & Point
- **Steger Line Center Extraction**
  - Sub-pixel Accuracy (<0.1 pixel)
  - Gaussian Smoothing
  - Hessian Matrix
  - Eigenvalue Analysis
  - Gap Handling

### 5.3 Seam Tracking
- **Real-time Seam Tracker**
  - Tracking Modes (Ahead, Coaxial)
  - Look-ahead Distance
  - Max Correction Limit
  - Enable/Disable
  - Tracking Loss Detection
- **Kalman Filter**
  - Position + Velocity State
  - Constant Velocity Model
  - Configurable Noise Parameters
  - Prediction on Data Loss
- **Latency Compensation**
  - Total Latency Measurement
  - Velocity-based Compensation
  - Auto-measure Function
- **Ruckig Integration**
  - Smooth Correction Application
  - Jerk-Limited Motion
  - Velocity/Acceleration Limits

### 5.4 Calibration
- **Hand-Eye Calibration (AX=XB)**
  - Methods: Tsai-Lenz, Park-Martin, Daniilidis, Andreff
  - Multiple Pose Capture
  - Error Metrics (Rotation, Translation)
  - Validation Routine
- **TCP Calibration (4-Point Method)**
  - Methods: Four-Point, Rotation Axis, Sphere Fit
  - Interactive Mode
  - Error Display (Sphere Radius)
- **Sensor Calibration Wizard**
  - Step-by-Step UI
  - Real-time Preview
  - Save/Load Calibration Data
- **Calibration Data Management**
  - YAML File Format
  - Versioning & Timestamp
  - Data Integrity Validation

### 5.5 Touch Sensing
- **Touch Sense Circuit Interface**
  - 24V DC Detection
  - Debounced Touch Detection
  - Retract Distance Configuration
  - Touch/Touch Lost Callbacks
- **Touch Search Patterns**
  - Linear Search (1D)
  - Cross Pattern (2D)
  - Spiral Search (2D)
  - Zigzag Pattern (Surface Scan)
  - Safe Motion with Slow Speed
  - Timeout Handling
- **Touch Offset Calibration**
  - 3D Offset (Wire Tip → TCP)
  - Multi-touch Sphere Fit
  - Accuracy <0.3mm

### 5.6 Vision HMI
- **Point Cloud Visualization**
  - Render 100k+ Points @ 30fps
  - Color by Intensity/Height
  - Seam Path Overlay
  - Coordinate Frames
  - Interactive Camera
  - Export to PLY/XYZ
- **Laser Profile View (2D)**
  - Real-time @ 100Hz
  - Detected Features Overlay
  - Zoom/Scroll
  - Record/Playback
- **Tracking Monitor Dashboard**
  - Real-time Correction Display
  - Error History Graph
  - Status Indicators
  - Configuration Controls
- **Sensor Status Panel**
  - Connection Status
  - Frame Rate
  - Data Quality
- **Profile Display Control**
  - 2D Profile Rendering
  - Feature Highlighting

---

## 6. APPLICATION MODES

### 6.1 Welding Mode (Primary - P0)
- Arc Welding Control
- Pre-flow/Post-flow
- Crater Fill
- Burnback
- Weaving Patterns
- Seam Tracking

### 6.2 Pick & Place Mode (Future - P2)
- Gripper Control
- Vision Guidance
- Palletizing

### 6.3 Scan-to-Path Mode (Future - P2)
- 3D Scanning
- Point Cloud Processing
- Automatic Path Generation
- Offline Programming

---

## 7. SAFETY & COMPLIANCE

### 7.1 Standards Compliance
- ISO 10218-1 (Safety Requirements for Industrial Robots)
- ISO 13849-1 (Safety-related Parts of Control Systems)
- IEC 60204-1 (Safety of Machinery - Electrical Equipment)

### 7.2 Safety Features Summary
- Dual-Channel Safety Inputs
- E-Stop with Category 0 Stop
- Deadman Switch (3-Position)
- Safety Door/Light Curtain Support
- Hard Limits (Firmware)
- Soft Limits (Software)
- Velocity Monitoring
- Heartbeat Monitoring (150ms)
- Safe Torque Off (STO) Relay

---

## 8. DEPLOYMENT & CONFIGURATION

### 8.1 Single-PC Deployment
- Windows 10/11
- Industrial PC Specs (i5/i7, 16GB RAM, SSD)
- Touch Screen Support
- USB 3.0 for MCU

### 8.2 File System Layout
- bin/ (Executables, DLLs)
- config/ (YAML/JSON configs)
- programs/ (Robot programs)
- models/ (3D STL files)
- logs/ (Rotating logs)
- firmware/ (HEX files)
- docs/ (User manuals)

---

## 9. TESTING FEATURES

### 9.1 Unit Testing
- Google Test (C++)
- xUnit (C#)
- Mock Hardware

### 9.2 Integration Testing
- IPC Communication Tests
- Motion System Tests
- Welding Sequence Tests
- Vision Pipeline Tests

### 9.3 Hardware-in-Loop Testing
- Oscilloscope Timing Validation
- Arc Simulator
- Calibration Blocks

---

## SUMMARY

| Category | Feature Count |
|----------|---------------|
| Core Platform | ~25 |
| Motion Control | ~45 |
| HMI | ~30 |
| Welding | ~40 |
| Vision | ~50 |
| Safety | ~15 |
| Deployment | ~10 |
| **Total** | **~215 features** |

---

*Document generated from project documentation analysis.*
