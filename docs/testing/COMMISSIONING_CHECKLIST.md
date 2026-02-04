# COMMISSIONING CHECKLIST

| Metadata      | Value                           |
|---------------|---------------------------------|
| Document      | Hardware Commissioning          |
| Version       | 1.0                             |
| Status        | DRAFT                           |
| Last Updated  | 2026-02-01                      |

---

## 1. Overview

### 1.1 Purpose

Document này cung cấp checklist đầy đủ cho việc commissioning (đưa vào vận hành) hệ thống Robot Controller trên hardware thực. Checklist được thiết kế để đảm bảo an toàn và vận hành đúng cách.

### 1.2 Prerequisites

Trước khi bắt đầu commissioning:
- [ ] Tất cả Phase tests đã PASS
- [ ] Simulation Mode đã test đầy đủ
- [ ] Tài liệu hardware đã có
- [ ] Nhân sự đã được training

### 1.3 Safety Requirements

⚠️ **CRITICAL SAFETY REQUIREMENTS**

| Requirement | Description | Verified |
|-------------|-------------|----------|
| E-Stop accessible | E-Stop button within reach | [ ] |
| Safety fence | Physical barrier around robot | [ ] |
| Warning signs | "Robot Operating" signs posted | [ ] |
| Fire extinguisher | Within 5m of work area | [ ] |
| First aid kit | Available in area | [ ] |
| PPE | Safety glasses, shoes required | [ ] |
| Training | Operators trained | [ ] |

---

## 2. Pre-Commissioning Inspection

### 2.1 Mechanical Inspection

| # | Item | Criteria | Verified | Notes |
|---|------|----------|----------|-------|
| 1 | Robot mounting | Bolts torqued to spec | [ ] | |
| 2 | Base plate level | < 0.1mm/m deviation | [ ] | |
| 3 | Joint covers | All in place, secure | [ ] | |
| 4 | Cable routing | No pinch points | [ ] | |
| 5 | Tool mounting | Secure, correct orientation | [ ] | |
| 6 | End effector | Functional, no damage | [ ] | |
| 7 | Limit switches | Physically intact | [ ] | |
| 8 | Lubrication | Joints lubricated per manual | [ ] | |

### 2.2 Electrical Inspection

| # | Item | Criteria | Verified | Notes |
|---|------|----------|----------|-------|
| 1 | Main power | Correct voltage (±5%) | [ ] | Measured: ___V |
| 2 | Grounding | < 1Ω to ground | [ ] | Measured: ___Ω |
| 3 | Motor cables | Correct phase order | [ ] | |
| 4 | Encoder cables | Shielded, connected | [ ] | |
| 5 | Control cables | Secure connections | [ ] | |
| 6 | E-Stop wiring | NC contacts, tested | [ ] | |
| 7 | Safety circuit | Verified per diagram | [ ] | |
| 8 | I/O wiring | Per I/O map | [ ] | |

### 2.3 Pneumatic/Hydraulic (if applicable)

| # | Item | Criteria | Verified | Notes |
|---|------|----------|----------|-------|
| 1 | Air supply | Correct pressure (___bar) | [ ] | Measured: ___bar |
| 2 | Air quality | Clean, dry (no moisture) | [ ] | |
| 3 | Connections | No leaks | [ ] | |
| 4 | Regulators | Set correctly | [ ] | |
| 5 | Lubricator | Filled, functioning | [ ] | |

---

## 3. Controller Setup

### 3.1 PC Hardware

| # | Item | Criteria | Verified | Notes |
|---|------|----------|----------|-------|
| 1 | PC power | UPS connected | [ ] | |
| 2 | USB connections | Core to Teensy | [ ] | COM___: |
| 3 | Network | IP configured | [ ] | IP: ___.___.___.___  |
| 4 | Sensor connections | All connected | [ ] | |
| 5 | I/O module | Connected, powered | [ ] | |

### 3.2 Software Installation

| # | Item | Criteria | Verified | Notes |
|---|------|----------|----------|-------|
| 1 | OS | Windows 10/11 64-bit | [ ] | Version: |
| 2 | .NET Runtime | 8.0 installed | [ ] | |
| 3 | VC++ Runtime | Latest installed | [ ] | |
| 4 | Application | Installed to correct path | [ ] | |
| 5 | Config files | Present and correct | [ ] | |
| 6 | Firmware | Correct version on Teensy | [ ] | Version: |

### 3.3 Configuration Verification

| # | Item | Check | Verified |
|---|------|-------|----------|
| 1 | Robot model | Correct URDF loaded | [ ] |
| 2 | Joint limits | Match actual robot | [ ] |
| 3 | DH parameters | Verified against robot | [ ] |
| 4 | I/O mapping | Matches wiring | [ ] |
| 5 | Communication | Correct ports/addresses | [ ] |

---

## 4. Power-On Sequence

### 4.1 First Power-On

**⚠️ Ensure E-Stop is PRESSED before power-on**

| Step | Action | Expected | Verified |
|------|--------|----------|----------|
| 1 | Connect main power | Power LED on | [ ] |
| 2 | Check for alarms | No immediate faults | [ ] |
| 3 | Check servo power | Servos NOT enabled (E-Stop in) | [ ] |
| 4 | Start control PC | Boots normally | [ ] |
| 5 | Launch application | UI appears | [ ] |
| 6 | Check connection status | "Connected" shown | [ ] |
| 7 | Check for errors | No error messages | [ ] |

### 4.2 E-Stop Verification

| Step | Action | Expected | Verified |
|------|--------|----------|----------|
| 1 | E-Stop pressed | Servos disabled | [ ] |
| 2 | Try to enable | Enable rejected | [ ] |
| 3 | Release E-Stop | System in IDLE | [ ] |
| 4 | Reset system | Ready to enable | [ ] |
| 5 | Press E-Stop while running | Immediate stop | [ ] |

**Witness Required:**
- Name: _______________
- Signature: _______________
- Date: _______________

---

## 5. Motor and Drive Check

### 5.1 Individual Axis Check

**⚠️ Reduce speed to 10% for initial tests**

| Axis | Direction | Motion Smooth | Correct Direction | No Noise | Verified |
|------|-----------|---------------|-------------------|----------|----------|
| J1 | + | [ ] | [ ] | [ ] | [ ] |
| J1 | - | [ ] | [ ] | [ ] | [ ] |
| J2 | + | [ ] | [ ] | [ ] | [ ] |
| J2 | - | [ ] | [ ] | [ ] | [ ] |
| J3 | + | [ ] | [ ] | [ ] | [ ] |
| J3 | - | [ ] | [ ] | [ ] | [ ] |
| J4 | + | [ ] | [ ] | [ ] | [ ] |
| J4 | - | [ ] | [ ] | [ ] | [ ] |
| J5 | + | [ ] | [ ] | [ ] | [ ] |
| J5 | - | [ ] | [ ] | [ ] | [ ] |
| J6 | + | [ ] | [ ] | [ ] | [ ] |
| J6 | - | [ ] | [ ] | [ ] | [ ] |

### 5.2 Encoder Check

| Axis | Positive Motion | Position Increases | Verified |
|------|-----------------|-------------------|----------|
| J1 | CCW from above | [ ] | [ ] |
| J2 | Forward | [ ] | [ ] |
| J3 | Forward | [ ] | [ ] |
| J4 | CCW from TCP | [ ] | [ ] |
| J5 | Forward | [ ] | [ ] |
| J6 | CCW from TCP | [ ] | [ ] |

---

## 6. Homing Procedure

### 6.1 Reference Point Setup

| Axis | Home Position | Method | Verified |
|------|---------------|--------|----------|
| J1 | 0° (forward) | Index pulse + limit | [ ] |
| J2 | 0° (vertical) | Index pulse + limit | [ ] |
| J3 | 0° (horizontal) | Index pulse + limit | [ ] |
| J4 | 0° | Index pulse | [ ] |
| J5 | 0° (horizontal) | Index pulse | [ ] |
| J6 | 0° | Index pulse | [ ] |

### 6.2 Homing Execution

| Step | Action | Expected | Verified |
|------|--------|----------|----------|
| 1 | Start homing sequence | All axes begin homing | [ ] |
| 2 | J1 homes | Reaches home, indicator ON | [ ] |
| 3 | J2 homes | Reaches home, indicator ON | [ ] |
| 4 | J3 homes | Reaches home, indicator ON | [ ] |
| 5 | J4 homes | Reaches home, indicator ON | [ ] |
| 6 | J5 homes | Reaches home, indicator ON | [ ] |
| 7 | J6 homes | Reaches home, indicator ON | [ ] |
| 8 | Homing complete | "Homed" status shown | [ ] |

### 6.3 Home Position Verification

| Check | Method | Expected | Actual | Pass |
|-------|--------|----------|--------|------|
| Visual alignment | Compare to reference marks | Aligned | | [ ] |
| Position readout | UI display | All zeros | | [ ] |
| TCP position | Measure from known point | Within 1mm | | [ ] |

---

## 7. I/O Verification

### 7.1 Digital Inputs

| Input | Function | Test Method | Result | Verified |
|-------|----------|-------------|--------|----------|
| DI0 | E-Stop | Press button | Signal LOW | [ ] |
| DI1 | Deadman | Press/release | Toggles | [ ] |
| DI2 | Safety fence | Open door | Signal changes | [ ] |
| DI3 | Part present | Sensor test | Toggles | [ ] |
| ... | ... | ... | ... | [ ] |

### 7.2 Digital Outputs

| Output | Function | Test Method | Result | Verified |
|--------|----------|-------------|--------|----------|
| DO0 | Arc enable | Command ON | Relay clicks | [ ] |
| DO1 | Gas valve | Command ON | Valve opens | [ ] |
| DO2 | Wire feed | Command ON | Feeder runs | [ ] |
| DO3 | Signal light | Command ON | Light ON | [ ] |
| ... | ... | ... | ... | [ ] |

### 7.3 Analog I/O

| Channel | Function | Test Value | Expected | Measured | Pass |
|---------|----------|------------|----------|----------|------|
| AO0 | Voltage cmd | 50% | 5.0V | ___V | [ ] |
| AO1 | Current cmd | 50% | 5.0V | ___V | [ ] |
| AI0 | Voltage FB | Apply 5V | 50% | ___% | [ ] |
| AI1 | Current FB | Apply 5V | 50% | ___% | [ ] |

---

## 8. Kinematics Verification

### 8.1 TCP Calibration

| Step | Action | Verified |
|------|--------|----------|
| 1 | Mount calibration tool | [ ] |
| 2 | Touch reference point from 4 orientations | [ ] |
| 3 | System calculates TCP offset | [ ] |
| 4 | Verify TCP accuracy | [ ] |

**TCP Offset Results:**
- X: ___mm
- Y: ___mm
- Z: ___mm
- Rx: ___°
- Ry: ___°
- Rz: ___°

### 8.2 Position Accuracy Test

| Point | Programmed | Measured | Error | Pass |
|-------|------------|----------|-------|------|
| P1 | X=___ Y=___ Z=___ | X=___ Y=___ Z=___ | ___mm | [ ] < 1mm |
| P2 | X=___ Y=___ Z=___ | X=___ Y=___ Z=___ | ___mm | [ ] < 1mm |
| P3 | X=___ Y=___ Z=___ | X=___ Y=___ Z=___ | ___mm | [ ] < 1mm |
| P4 | X=___ Y=___ Z=___ | X=___ Y=___ Z=___ | ___mm | [ ] < 1mm |

### 8.3 Repeatability Test

Move to same point 10 times:

| Trial | X (mm) | Y (mm) | Z (mm) |
|-------|--------|--------|--------|
| 1 | | | |
| 2 | | | |
| 3 | | | |
| 4 | | | |
| 5 | | | |
| 6 | | | |
| 7 | | | |
| 8 | | | |
| 9 | | | |
| 10 | | | |
| **Std Dev** | | | |

**Pass Criteria:** Std dev < 0.1mm in all axes

---

## 9. Safety System Verification

### 9.1 E-Stop Test

| Test | Condition | Expected Response | Response Time | Pass |
|------|-----------|-------------------|---------------|------|
| From jog | Robot jogging | Complete stop | < 500ms | [ ] |
| From program | Executing program | Complete stop | < 500ms | [ ] |
| From weld | Welding active | Arc off, motion stop | < 500ms | [ ] |
| Multiple E-Stops | Press second E-Stop | System stays stopped | Immediate | [ ] |

**Witness Required:**
- Name: _______________
- Signature: _______________

### 9.2 Safety Fence

| Test | Expected | Verified |
|------|----------|----------|
| Door open while stopped | Allow reset | [ ] |
| Door open while running | Protective stop | [ ] |
| Cannot start with door open | Start blocked | [ ] |
| Door close after stop | Can resume | [ ] |

### 9.3 Joint Limits

| Axis | Positive Limit | Negative Limit | Soft Limit Works | Verified |
|------|----------------|----------------|------------------|----------|
| J1 | ___° | ___° | [ ] | [ ] |
| J2 | ___° | ___° | [ ] | [ ] |
| J3 | ___° | ___° | [ ] | [ ] |
| J4 | ___° | ___° | [ ] | [ ] |
| J5 | ___° | ___° | [ ] | [ ] |
| J6 | ___° | ___° | [ ] | [ ] |

---

## 10. Application-Specific Setup

### 10.1 Welding System (if applicable)

| Item | Setting | Verified |
|------|---------|----------|
| Welding source connected | Model: ___ | [ ] |
| Interface cable | Connected | [ ] |
| Gas supply | Pressure: ___bar | [ ] |
| Wire feeder | Wire loaded: ___mm | [ ] |
| Torch mounted | Secure | [ ] |
| Ground clamp | Connected to workpiece | [ ] |

### 10.2 Vision System (if applicable)

| Item | Setting | Verified |
|------|---------|----------|
| Camera/Sensor mounted | Secure | [ ] |
| Cable connected | Ethernet/USB | [ ] |
| IP configured | ___.___.___.___ | [ ] |
| Hand-eye calibrated | Error < 1mm | [ ] |

---

## 11. Functional Tests

### 11.1 Jog Mode Test

| Test | Expected | Verified |
|------|----------|----------|
| Joint jog all axes | Smooth motion | [ ] |
| Cartesian jog XYZ | Correct direction | [ ] |
| Cartesian jog rotation | Correct direction | [ ] |
| Speed override 10% | Reduced speed | [ ] |
| Speed override 100% | Full speed | [ ] |
| Deadman required | Cannot jog without | [ ] |

### 11.2 Program Execution

| Test | Expected | Verified |
|------|----------|----------|
| Load test program | Loads successfully | [ ] |
| Dry run (no process) | Correct path | [ ] |
| Single step mode | Stops at each point | [ ] |
| Continuous mode | Runs without stopping | [ ] |
| Pause/Resume | Works correctly | [ ] |
| Stop | Immediate stop | [ ] |

### 11.3 Production Test

| Test | Expected | Verified |
|------|----------|----------|
| Full production cycle | Completes successfully | [ ] |
| Cycle time | Within specification | [ ] |
| Quality acceptable | Meets standards | [ ] |
| Repeat 3 cycles | Consistent results | [ ] |

---

## 12. Documentation

### 12.1 As-Built Records

| Document | Status | Location |
|----------|--------|----------|
| Wiring diagrams | Updated | [ ] |
| I/O list | Verified | [ ] |
| Parameter settings | Backed up | [ ] |
| TCP calibration data | Recorded | [ ] |
| Safety settings | Documented | [ ] |

### 12.2 Operator Training

| Trainee | Topics Covered | Date | Trainer |
|---------|----------------|------|---------|
| | Basic operation, E-Stop | | |
| | Program execution | | |
| | Troubleshooting | | |

---

## 13. Sign-Off

### 13.1 Commissioning Completion

**All items on this checklist have been verified and the system is ready for production.**

| Role | Name | Signature | Date |
|------|------|-----------|------|
| Commissioning Engineer | | | |
| Safety Officer | | | |
| Project Manager | | | |
| Customer Representative | | | |

### 13.2 Punch List (if applicable)

Items to be completed after initial commissioning:

| # | Item | Responsible | Due Date | Status |
|---|------|-------------|----------|--------|
| 1 | | | | |
| 2 | | | | |
| 3 | | | | |

---

## 14. Document History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2026-02-01 | System | Initial version |
