# PHASE 3: WELDING TECHNOLOGY
## "Robot Welds" - Hệ thống Hàn Hoàn Chỉnh

**Version:** 1.0
**Ngày tạo:** 01/02/2026
**Trạng thái:** DRAFT
**Phụ thuộc:** Phase 2 (Motion Core) phải hoàn thành

---

## 1. TỔNG QUAN & PHẠM VI

### 1.1 Mục tiêu Phase 3
Xây dựng hệ thống điều khiển hàn MIG/MAG hoàn chỉnh, cho phép robot thực hiện quy trình hàn tự động với khả năng:
- Điều khiển nguồn hàn theo trình tự chuẩn công nghiệp
- Tạo các mẫu weaving (hàn lắc) đa dạng
- Lập trình và lưu trữ chương trình hàn
- Giám sát và xử lý lỗi trong quá trình hàn

### 1.2 Milestone: "Robot Welds"
**Tiêu chí hoàn thành:**
- Robot thực hiện đường hàn thẳng 100mm với weaving pattern
- Gas pre-flow, arc ignition, weld, crater fill, post-flow đúng trình tự
- UI hiển thị real-time: dòng điện, điện áp, trạng thái arc
- Xử lý được lỗi ignition failure và wire stick

### 1.3 Kiến trúc Master-Slave

```
┌─────────────────────────────────────────────────────────────────┐
│                    ROBOT CONTROLLER (MASTER)                     │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐   │
│  │ Weld Program │  │  Trajectory  │  │   WeldingSequencer   │   │
│  │   Manager    │──│   Planner    │──│        (FSM)         │   │
│  └──────────────┘  └──────────────┘  └──────────┬───────────┘   │
│                                                  │               │
│  ┌──────────────────────────────────────────────┴──────────────┐│
│  │                    I/O Interface Layer                       ││
│  │  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────────────┐ ││
│  │  │Digital  │  │Digital  │  │Analog   │  │   Signal        │ ││
│  │  │Outputs  │  │Inputs   │  │Outputs  │  │   Debouncing    │ ││
│  │  └────┬────┘  └────┬────┘  └────┬────┘  └─────────────────┘ ││
│  └───────┼───────────┼────────────┼────────────────────────────┘│
└──────────┼───────────┼────────────┼─────────────────────────────┘
           │           │            │
           ▼           ▼            ▼
┌─────────────────────────────────────────────────────────────────┐
│                  WELDING POWER SOURCE (SLAVE)                    │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │                    Synergic Core                          │   │
│  │  • Nhận WFS Reference (0-10V) → Tính toán V, I phù hợp   │   │
│  │  • Tự điều chỉnh theo thuật toán synergic                │   │
│  │  • Trả về ARC_OK, READY, ERROR signals                   │   │
│  └──────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

**Nguyên tắc phân chia trách nhiệm:**
| Thành phần | Trách nhiệm |
|------------|-------------|
| Robot Controller | Timing, Sequencing, Motion, Weaving |
| Power Source | Synergic curves, Arc physics, Current/Voltage control |

---

## 2. CẤU TRÚC THƯ MỤC

### 2.1 Thư mục mới cần tạo

```
src/
├── core/
│   ├── welding/                    # [NEW] Module hàn chính
│   │   ├── WeldingSequencer.hpp    # FSM điều khiển trình tự hàn
│   │   ├── WeldingSequencer.cpp
│   │   ├── WeldingConfig.hpp       # Cấu hình timing parameters
│   │   ├── WeldingState.hpp        # Enum các trạng thái
│   │   └── ArcMonitor.hpp          # Giám sát arc stability
│   │
│   ├── weaving/                    # [NEW] Module hàn lắc
│   │   ├── WeldPatternGenerator.hpp
│   │   ├── WeldPatternGenerator.cpp
│   │   ├── patterns/
│   │   │   ├── SineWave.hpp
│   │   │   ├── TriangleWave.hpp
│   │   │   ├── TrapezoidWave.hpp
│   │   │   ├── CirclePattern.hpp
│   │   │   ├── Figure8Pattern.hpp
│   │   │   └── LTypePattern.hpp
│   │   └── WeavingFrame.hpp        # Tính toán Tangent/Binormal
│   │
│   ├── io/                         # [NEW] I/O Interface
│   │   ├── WeldingIO.hpp           # Abstract interface
│   │   ├── WeldingIO.cpp
│   │   ├── DigitalIO.hpp           # Digital I/O handler
│   │   ├── AnalogIO.hpp            # Analog 0-10V handler
│   │   └── SignalDebouncer.hpp     # Debouncing logic
│   │
│   └── program/                    # [NEW] Weld Program Manager
│       ├── WeldProgram.hpp         # Cấu trúc chương trình hàn
│       ├── WeldProgramManager.hpp  # Load/Save/Execute
│       ├── WeldInstruction.hpp     # Các lệnh hàn
│       └── WeldJobScheduler.hpp    # Quản lý jobs
│
├── ui/
│   └── welding/                    # [NEW] UI cho hàn
│       ├── WeldingPanel.xaml       # Panel điều khiển hàn
│       ├── WeldingPanel.xaml.cs
│       ├── WeldMonitorView.xaml    # Hiển thị dòng/áp real-time
│       ├── WeavingSetupView.xaml   # Cấu hình weaving
│       └── WeldProgramEditor.xaml  # Editor chương trình hàn
│
└── tests/
    └── welding/                    # [NEW] Unit tests
        ├── test_WeldingSequencer.cpp
        ├── test_WeldPatternGenerator.cpp
        └── test_WeldingIO.cpp
```

---

## 3. DANH SÁCH TASK CHI TIẾT

### 3.1 Nhóm A: I/O Interface Layer (P3-01 → P3-04)

#### P3-01: Digital Output Interface
**Mô tả:** Xây dựng lớp điều khiển Digital Output cho các tín hiệu điều khiển nguồn hàn.

**Tín hiệu Digital Output:**
| Signal | Pin | Chức năng | Active Level |
|--------|-----|-----------|--------------|
| DO_GAS_VALVE | DO.0 | Van khí bảo vệ | HIGH = Mở |
| DO_ARC_START | DO.1 | Lệnh bắt đầu hàn | HIGH = Start |
| DO_WIRE_INCH | DO.2 | Đẩy dây (không hàn) | HIGH = Đẩy |
| DO_WIRE_RETRACT | DO.3 | Rút dây | HIGH = Rút |
| DO_TORCH_COLLISION | DO.4 | Reset collision | Pulse |

**Interface:**
```cpp
class IDigitalOutput {
public:
    virtual void setGasValve(bool state) = 0;
    virtual void setArcStart(bool state) = 0;
    virtual void setWireInch(bool state) = 0;
    virtual void setWireRetract(bool state) = 0;
    virtual void pulseCollisionReset(uint32_t ms) = 0;
};
```

**DoD:**
- [ ] Điều khiển được tất cả 5 tín hiệu DO
- [ ] Unit test với mock hardware
- [ ] Logging mọi thay đổi trạng thái

---

#### P3-02: Digital Input Interface
**Mô tả:** Xây dựng lớp đọc Digital Input từ nguồn hàn với debouncing.

**Tín hiệu Digital Input:**
| Signal | Pin | Chức năng | Debounce |
|--------|-----|-----------|----------|
| DI_ARC_OK | DI.0 | Arc ổn định | 20ms HIGH, 50ms LOW |
| DI_READY | DI.1 | Nguồn hàn sẵn sàng | 10ms |
| DI_WELD_ERROR | DI.2 | Lỗi từ nguồn hàn | 10ms |
| DI_WIRE_STUCK | DI.3 | Dây bị kẹt | 10ms |
| DI_GAS_PRESSURE_OK | DI.4 | Áp suất khí OK | 50ms |

**Debouncing Logic cho DI_ARC_OK:**
```cpp
// Arc OK cần debounce bất đối xứng:
// - Rising edge: 20ms HIGH liên tục mới xác nhận ARC_OK = true
// - Falling edge: 50ms LOW liên tục mới xác nhận ARC_OK = false
// Lý do: Tránh false positive khi arc dao động
```

**DoD:**
- [ ] Đọc được tất cả 5 tín hiệu DI
- [ ] Debouncing hoạt động đúng với timing khác nhau
- [ ] Unit test debouncing với simulated noise

---

#### P3-03: Analog Output Interface
**Mô tả:** Xây dựng lớp điều khiển Analog Output 0-10V cho điều khiển nguồn hàn.

**Tín hiệu Analog Output:**
| Signal | Range | Chức năng | Resolution |
|--------|-------|-----------|------------|
| AO_WFS_REF | 0-10V | Wire Feed Speed Reference | 12-bit |
| AO_VOLT_REF | 0-10V | Voltage Reference (nếu có) | 12-bit |

**Mapping:**
```cpp
// WFS: 0V = 0 m/min, 10V = 25 m/min (tùy nguồn)
float wfsToVoltage(float wfs_m_min) {
    return std::clamp(wfs_m_min / 25.0f * 10.0f, 0.0f, 10.0f);
}

// Voltage: 0V = 10V arc, 10V = 40V arc (tùy nguồn)
float arcVoltToRefVolt(float arc_volt) {
    return std::clamp((arc_volt - 10.0f) / 30.0f * 10.0f, 0.0f, 10.0f);
}
```

**DoD:**
- [ ] Output 0-10V với độ phân giải 12-bit
- [ ] Calibration routine cho offset/gain
- [ ] Smooth ramping để tránh spike

---

#### P3-04: Unified WeldingIO Class
**Mô tả:** Tạo lớp tổng hợp quản lý tất cả I/O liên quan đến hàn.

**Interface:**
```cpp
class WeldingIO {
public:
    // Digital Outputs
    void setGasValve(bool state);
    void setArcStart(bool state);
    void setWireInch(bool state);

    // Digital Inputs (debounced)
    bool isArcOK() const;
    bool isReady() const;
    bool hasError() const;
    bool isWireStuck() const;
    bool isGasPressureOK() const;

    // Analog Outputs
    void setWireFeedSpeed(float m_per_min);
    void setVoltageReference(float volts);

    // Update cycle (call in main loop)
    void update(uint32_t dt_ms);

private:
    SignalDebouncer m_arcOkDebouncer{20, 50}; // asymmetric
    SignalDebouncer m_readyDebouncer{10, 10};
    // ...
};
```

**DoD:**
- [ ] Tích hợp tất cả P3-01, P3-02, P3-03
- [ ] Thread-safe cho multi-thread access
- [ ] Event callback khi tín hiệu thay đổi

---

### 3.2 Nhóm B: WeldingSequencer FSM (P3-05 → P3-09)

#### P3-05: WeldingState Enum & Transitions
**Mô tả:** Định nghĩa các trạng thái và quy tắc chuyển đổi của FSM.

**State Diagram:**
```
                    ┌──────────────────────────────────────┐
                    │              FAULT                    │
                    │  (Ignition Fail / Wire Stick / E-Stop)│
                    └──────────────────────────────────────┘
                           ▲              ▲              ▲
                           │              │              │
    ┌────────┐         ┌───┴────┐    ┌────┴───┐    ┌─────┴────┐
    │  IDLE  │────────▶│PREFLOW │───▶│IGNITION│───▶│STABILIZE │
    └────────┘  Start  └────────┘    └────────┘    └──────────┘
         ▲              Gas ON       ARC_START      Wait Arc OK
         │                           + Wire         stable
         │                                               │
         │     ┌────────┐    ┌────────┐    ┌────────┐    │
         │     │POSTFLOW│◀───│BURNBACK│◀───│ CRATER │◀───┘
         │     └────────┘    └────────┘    └────────┘
         │      Gas OFF       Wire burn    Crater fill
         │         │          back
         └─────────┘
            Complete
```

**Enum Definition:**
```cpp
enum class WeldingState : uint8_t {
    IDLE = 0,       // Chờ lệnh
    PREFLOW,        // Xả khí trước
    IGNITION,       // Châm hồ quang
    STABILIZE,      // Chờ arc ổn định
    WELD,           // Đang hàn
    CRATER,         // Crater fill (giảm dần)
    BURNBACK,       // Đốt cháy dây thừa
    POSTFLOW,       // Xả khí sau
    FAULT           // Lỗi
};
```

**DoD:**
- [ ] Enum và transition matrix định nghĩa đầy đủ
- [ ] Diagram trong documentation
- [ ] Validation function cho mọi transition

---

#### P3-06: WeldingConfig & Timing Parameters
**Mô tả:** Định nghĩa cấu trúc cấu hình và các tham số timing.

**Timing Parameters:**
| Parameter | Default | Range | Unit | Mô tả |
|-----------|---------|-------|------|-------|
| PreFlowTime | 500 | 100-2000 | ms | Thời gian xả khí trước |
| IgnitionTimeout | 2000 | 500-5000 | ms | Timeout chờ arc |
| StabilizeTime | 200 | 50-1000 | ms | Thời gian chờ arc ổn định |
| CraterTime | 500 | 0-2000 | ms | Thời gian crater fill |
| BurnbackTime | 100 | 0-500 | ms | Thời gian đốt dây thừa |
| PostFlowTime | 1000 | 200-5000 | ms | Thời gian xả khí sau |

**Config Structure:**
```cpp
struct WeldingConfig {
    // Timing
    uint32_t preFlowTime_ms = 500;
    uint32_t ignitionTimeout_ms = 2000;
    uint32_t stabilizeTime_ms = 200;
    uint32_t craterTime_ms = 500;
    uint32_t burnbackTime_ms = 100;
    uint32_t postFlowTime_ms = 1000;

    // Welding parameters
    float wireFeedSpeed_m_min = 5.0f;
    float voltage_V = 20.0f;

    // Crater fill (percentage reduction)
    float craterWfsReduction = 0.3f;  // 30% reduction
    float craterVoltReduction = 0.2f; // 20% reduction

    // Retry
    uint8_t ignitionRetries = 2;
};
```

**DoD:**
- [ ] Load/Save config từ JSON/YAML
- [ ] Validation cho tất cả parameters
- [ ] Default config file mẫu

---

#### P3-07: WeldingSequencer Core Implementation
**Mô tả:** Implement FSM logic chính của WeldingSequencer.

**Class Interface:**
```cpp
class WeldingSequencer {
public:
    WeldingSequencer(WeldingIO& io, const WeldingConfig& config);

    // Commands
    bool startWeld();
    void stopWeld();       // Graceful stop (go to CRATER)
    void emergencyStop();  // Immediate stop (go to POSTFLOW)
    void reset();          // Clear FAULT, return to IDLE

    // Update (call every cycle, e.g., 10ms)
    void update(uint32_t dt_ms);

    // State queries
    WeldingState getState() const;
    uint32_t getStateTime() const;  // Time in current state
    FaultCode getLastFault() const;

    // Callbacks
    void onStateChange(std::function<void(WeldingState, WeldingState)> cb);
    void onFault(std::function<void(FaultCode)> cb);

private:
    void enterState(WeldingState newState);
    void updateIDLE();
    void updatePREFLOW();
    void updateIGNITION();
    void updateSTABILIZE();
    void updateWELD();
    void updateCRATER();
    void updateBURNBACK();
    void updatePOSTFLOW();
    void updateFAULT();

    WeldingIO& m_io;
    WeldingConfig m_config;
    WeldingState m_state = WeldingState::IDLE;
    uint32_t m_stateTimer = 0;
    uint8_t m_ignitionRetries = 0;
};
```

**DoD:**
- [ ] Tất cả state transitions hoạt động đúng
- [ ] Timing chính xác ±10ms
- [ ] Unit tests cho mọi state path

---

#### P3-08: Fault Detection & Recovery
**Mô tả:** Implement logic phát hiện và xử lý lỗi.

**Fault Codes:**
```cpp
enum class FaultCode : uint8_t {
    NONE = 0,
    IGNITION_FAILURE,    // Không thể châm arc sau N lần thử
    ARC_LOST,            // Mất arc trong khi hàn
    WIRE_STUCK,          // Dây bị kẹt
    GAS_PRESSURE_LOW,    // Áp suất khí thấp
    POWER_SOURCE_ERROR,  // Lỗi từ nguồn hàn
    COMMUNICATION_ERROR, // Mất kết nối I/O
    EMERGENCY_STOP       // E-Stop triggered
};
```

**Recovery Matrix:**
| Fault | Recoverable | Action |
|-------|-------------|--------|
| IGNITION_FAILURE | Yes | Reset + Retry |
| ARC_LOST | Yes | Reset + Re-teach |
| WIRE_STUCK | No | Manual intervention |
| GAS_PRESSURE_LOW | Yes | Wait for pressure |
| POWER_SOURCE_ERROR | Depends | Check power source |
| EMERGENCY_STOP | Yes | Reset E-Stop + Reset |

**DoD:**
- [ ] Phát hiện tất cả fault codes
- [ ] Logging chi tiết khi fault xảy ra
- [ ] Recovery procedure cho recoverable faults

---

#### P3-09: Arc Monitor & Quality Metrics
**Mô tả:** Giám sát chất lượng hồ quang real-time.

**Metrics:**
```cpp
struct ArcMetrics {
    float arcOnTime_s;        // Tổng thời gian arc on
    float arcOffTime_s;       // Tổng thời gian arc off (trong WELD state)
    float arcStability;       // 0.0 - 1.0 (% time arc OK)
    uint32_t arcLostCount;    // Số lần mất arc
    float avgWireFeedSpeed;   // WFS trung bình
    float avgVoltage;         // Voltage trung bình (nếu đọc được)
};
```

**DoD:**
- [ ] Thu thập metrics real-time
- [ ] Export metrics sau mỗi weld cycle
- [ ] Alarm nếu arcStability < threshold

---

### 3.3 Nhóm C: Weaving Patterns (P3-10 → P3-13)

#### P3-10: WeavingFrame Calculation
**Mô tả:** Tính toán hệ tọa độ Weaving Frame dựa trên đường hàn.

**Weaving Frame:**
```
        Z (Torch axis)
        │
        │    Y (Binormal - weave direction)
        │   /
        │  /
        │ /
        └──────── X (Tangent - weld direction)

Công thức:
- Tangent (T) = normalize(P_next - P_current)
- Binormal (B) = normalize(T × Z_torch)
- Normal (N) = T × B
```

**Interface:**
```cpp
struct WeavingFrame {
    Eigen::Vector3d tangent;    // Hướng đường hàn
    Eigen::Vector3d binormal;   // Hướng dao động
    Eigen::Vector3d normal;     // Hướng pháp tuyến
    Eigen::Vector3d origin;     // Điểm gốc

    static WeavingFrame fromWeldPath(
        const Eigen::Vector3d& current,
        const Eigen::Vector3d& next,
        const Eigen::Vector3d& torchAxis
    );
};
```

**DoD:**
- [ ] Tính đúng frame cho đường thẳng
- [ ] Xử lý edge case: đường thẳng đứng
- [ ] Unit test với các hướng khác nhau

---

#### P3-11: Base Pattern Interface & Sine Wave
**Mô tả:** Định nghĩa interface chung và implement Sine Wave pattern.

**Base Interface:**
```cpp
class IWeavingPattern {
public:
    virtual ~IWeavingPattern() = default;

    // Get offset at given phase (0.0 - 1.0)
    virtual Eigen::Vector3d getOffset(
        double phase,
        const WeavingFrame& frame
    ) const = 0;

    // Pattern info
    virtual std::string getName() const = 0;
    virtual WeavingPatternType getType() const = 0;
};

struct WeavingParams {
    double amplitude_mm = 3.0;      // Biên độ dao động
    double frequency_Hz = 2.0;      // Tần số
    double dwellLeft_ms = 0;        // Dừng bên trái
    double dwellRight_ms = 0;       // Dừng bên phải
    double phaseOffset = 0.0;       // Phase offset (0-1)
};
```

**Sine Wave Formula:**
```cpp
// y(t) = A * sin(2π * f * t + φ)
Eigen::Vector3d SineWave::getOffset(double phase, const WeavingFrame& frame) const {
    double y = m_params.amplitude_mm * std::sin(2.0 * M_PI * phase);
    return frame.binormal * y;
}
```

**DoD:**
- [ ] Interface có thể mở rộng
- [ ] Sine wave chính xác
- [ ] Unit test với expected values

---

#### P3-12: Additional Patterns (Triangle, Trapezoid, Circle)
**Mô tả:** Implement các pattern phổ biến khác.

**Triangle Wave:**
```cpp
// Dạng tam giác: đi thẳng từ -A đến +A rồi quay lại
double TriangleWave::calculate(double phase) {
    double t = std::fmod(phase, 1.0);
    if (t < 0.25) return 4.0 * t * m_amplitude;
    if (t < 0.75) return (2.0 - 4.0 * t) * m_amplitude;
    return (4.0 * t - 4.0) * m_amplitude;
}
```

**Trapezoid Wave:**
```cpp
// Dạng hình thang: đi ngang ở 2 đầu, nghiêng ở giữa
// Gồm 4 phase: dwell_left, transition_to_right, dwell_right, transition_to_left
```

**Circle Pattern:**
```cpp
// Vòng tròn: x = A*cos(θ), y = A*sin(θ)
Eigen::Vector3d CirclePattern::getOffset(double phase, const WeavingFrame& frame) {
    double theta = 2.0 * M_PI * phase;
    double x = m_params.amplitude_mm * std::cos(theta);
    double y = m_params.amplitude_mm * std::sin(theta);
    return frame.tangent * x + frame.binormal * y;
}
```

**DoD:**
- [ ] 3 patterns hoạt động đúng
- [ ] Visualize patterns trong UI
- [ ] Unit tests cho mỗi pattern

---

#### P3-13: Advanced Patterns (Figure-8, L-Type) & Dwell Time
**Mô tả:** Implement patterns nâng cao và logic dwell time.

**Figure-8 (Lissajous):**
```cpp
// x = A * sin(2θ), y = A * sin(θ)
// Tạo hình số 8 nằm ngang
Eigen::Vector3d Figure8::getOffset(double phase, const WeavingFrame& frame) {
    double theta = 2.0 * M_PI * phase;
    double x = m_params.amplitude_mm * std::sin(2.0 * theta);
    double y = m_params.amplitude_mm * std::sin(theta);
    return frame.tangent * x + frame.binormal * y;
}
```

**L-Type Pattern:**
```cpp
// Dạng chữ L: dùng cho góc fillet
// Đi ngang, dừng, đi xuống, dừng, lặp lại
```

**Dwell Time Logic:**
```cpp
// Dwell time: dừng torch ở biên để penetration tốt hơn
double adjustedPhase = applyDwell(phase, m_params.dwellLeft_ms, m_params.dwellRight_ms);

double applyDwell(double phase, double dwellLeft, double dwellRight) {
    double totalDwell = dwellLeft + dwellRight;
    double cycleTime = 1.0 / m_params.frequency_Hz;
    double dwellRatio = totalDwell / (cycleTime * 1000.0);

    // Stretch phase to accommodate dwell
    // ... implementation
}
```

**DoD:**
- [ ] Figure-8 và L-Type hoạt động
- [ ] Dwell time chính xác
- [ ] Unit tests cho dwell timing

---

### 3.4 Nhóm D: Weld Pattern Generator (P3-14 → P3-15)

#### P3-14: WeldPatternGenerator Class
**Mô tả:** Tạo class tổng hợp để sinh weaving offsets theo thời gian.

**Interface:**
```cpp
class WeldPatternGenerator {
public:
    WeldPatternGenerator();

    // Configuration
    void setPattern(WeavingPatternType type);
    void setParams(const WeavingParams& params);
    void setWeldPath(const std::vector<Eigen::Vector3d>& path);

    // Enable/Disable
    void enable();
    void disable();
    bool isEnabled() const;

    // Update (call in trajectory loop)
    Eigen::Vector3d update(double dt_s, const Eigen::Vector3d& nominalPos);

    // Returns: nominalPos + weavingOffset

    // State
    double getPhase() const;
    double getAmplitude() const;

private:
    std::unique_ptr<IWeavingPattern> m_pattern;
    WeavingParams m_params;
    WeavingFrame m_frame;
    double m_phase = 0.0;
    bool m_enabled = false;
};
```

**DoD:**
- [ ] Hoạt động với tất cả patterns
- [ ] Smooth enable/disable (ramp up/down)
- [ ] Thread-safe

---

#### P3-15: Integration với Trajectory Planner
**Mô tả:** Tích hợp WeldPatternGenerator vào pipeline trajectory.

**Integration Point:**
```cpp
// Trong TrajectoryPlanner::update():
Eigen::Vector3d nominalPosition = m_ruckig.getPosition();

if (m_weldPatternGenerator.isEnabled()) {
    Eigen::Vector3d weavedPosition = m_weldPatternGenerator.update(dt, nominalPosition);
    // Gửi weavedPosition thay vì nominalPosition
} else {
    // Gửi nominalPosition
}
```

**DoD:**
- [ ] Weaving hoạt động mượt mà với MOVL
- [ ] Không gây jerk khi enable/disable
- [ ] Performance < 100μs per update

---

### 3.5 Nhóm E: Weld Program Manager (P3-16 → P3-17)

#### P3-16: WeldProgram & WeldInstruction Structures
**Mô tả:** Định nghĩa cấu trúc chương trình hàn.

**WeldInstruction Types:**
```cpp
enum class WeldInstructionType : uint8_t {
    // Motion
    MOVJ,           // Joint move (không hàn)
    MOVL,           // Linear move (không hàn)

    // Welding
    WELD_START,     // Bắt đầu hàn tại điểm hiện tại
    WELD_END,       // Kết thúc hàn
    WELD_LINE,      // Di chuyển + hàn (linear)
    WELD_ARC,       // Di chuyển + hàn (arc)

    // Settings
    SET_WFS,        // Set Wire Feed Speed
    SET_VOLTAGE,    // Set Voltage
    SET_WEAVING,    // Enable/config weaving

    // Control
    WAIT,           // Chờ (ms)
    COMMENT,        // Comment (không thực thi)
};

struct WeldInstruction {
    WeldInstructionType type;
    Eigen::VectorXd target;      // Target position (joint or Cartesian)
    WeldingConfig weldConfig;    // Welding parameters
    WeavingParams weavingParams; // Weaving parameters
    double speed;                // Speed (mm/s or %)
    std::string comment;
};

struct WeldProgram {
    std::string name;
    std::string description;
    std::vector<WeldInstruction> instructions;
    std::map<std::string, Eigen::VectorXd> positions;  // Named positions
    WeldingConfig defaultConfig;
};
```

**DoD:**
- [ ] Structures đầy đủ thông tin
- [ ] Serializable to/from JSON
- [ ] Validation function

---

#### P3-17: WeldProgramManager
**Mô tả:** Quản lý load/save/execute weld programs.

**Interface:**
```cpp
class WeldProgramManager {
public:
    // File operations
    bool loadProgram(const std::string& filepath);
    bool saveProgram(const std::string& filepath);
    bool saveAsProgram(const std::string& filepath);

    // Program management
    WeldProgram& getCurrentProgram();
    bool createNewProgram(const std::string& name);

    // Execution
    bool startExecution();
    void pauseExecution();
    void resumeExecution();
    void stopExecution();

    // State
    bool isExecuting() const;
    size_t getCurrentInstructionIndex() const;
    ExecutionState getExecutionState() const;

    // Teaching
    void teachPosition(const std::string& name);
    void insertInstruction(size_t index, const WeldInstruction& instr);
    void deleteInstruction(size_t index);

private:
    WeldProgram m_program;
    size_t m_currentIndex = 0;
    ExecutionState m_execState = ExecutionState::STOPPED;
};
```

**File Format (JSON):**
```json
{
  "name": "WeldJob_001",
  "description": "Hàn mối nối ống",
  "defaultConfig": {
    "wireFeedSpeed_m_min": 5.0,
    "voltage_V": 20.0,
    "preFlowTime_ms": 500
  },
  "positions": {
    "HOME": [0, -90, 90, 0, 90, 0],
    "START": [10, -85, 85, 5, 88, 10]
  },
  "instructions": [
    {"type": "MOVJ", "target": "HOME", "speed": 50},
    {"type": "MOVL", "target": "START", "speed": 100},
    {"type": "WELD_START"},
    {"type": "WELD_LINE", "target": [100, 0, 0], "speed": 5},
    {"type": "WELD_END"}
  ]
}
```

**DoD:**
- [ ] Load/Save JSON files
- [ ] Execute all instruction types
- [ ] Pause/Resume working

---

### 3.6 Nhóm F: Welding UI (P3-18 → P3-19)

#### P3-18: WeldingPanel UI
**Mô tả:** Tạo panel điều khiển hàn chính.

**UI Components:**
```
┌─────────────────────────────────────────────────────────────┐
│  WELDING CONTROL                                    [AUTO]  │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐  │
│  │  STATE      │  │  ARC        │  │  GAS               │  │
│  │  ■ IDLE     │  │  ○ OFF      │  │  ○ OFF             │  │
│  │  ○ PREFLOW  │  │  ● OK       │  │  ● ON              │  │
│  │  ○ WELD     │  │             │  │                    │  │
│  │  ○ POSTFLOW │  │             │  │                    │  │
│  └─────────────┘  └─────────────┘  └─────────────────────┘  │
├─────────────────────────────────────────────────────────────┤
│  PARAMETERS                                                  │
│  ┌─────────────────────────────────────────────────────────┐│
│  │ Wire Feed Speed    [■■■■■■■□□□]  8.5 m/min             ││
│  │ Voltage            [■■■■■■□□□□]  22.0 V                ││
│  │ Travel Speed       [■■■■□□□□□□]  5.0 mm/s              ││
│  └─────────────────────────────────────────────────────────┘│
├─────────────────────────────────────────────────────────────┤
│  WEAVING                                          [ENABLED]  │
│  ┌───────────┐ ┌───────────┐ ┌───────────┐ ┌─────────────┐  │
│  │ Pattern:  │ │ Amplitude │ │ Frequency │ │   Preview   │  │
│  │ [Sine ▼]  │ │  3.0 mm   │ │  2.0 Hz   │ │   ~~~|~~~   │  │
│  └───────────┘ └───────────┘ └───────────┘ └─────────────┘  │
├─────────────────────────────────────────────────────────────┤
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌─────────────┐  │
│  │  START   │  │   STOP   │  │  RESET   │  │  WIRE INCH  │  │
│  │    ▶     │  │    ■     │  │    ↺     │  │     ↑       │  │
│  └──────────┘  └──────────┘  └──────────┘  └─────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

**DoD:**
- [ ] Hiển thị state machine realtime
- [ ] Controls hoạt động đúng
- [ ] Weaving pattern preview

---

#### P3-19: WeldMonitorView & Diagnostics
**Mô tả:** Màn hình giám sát chi tiết quá trình hàn.

**UI Components:**
```
┌─────────────────────────────────────────────────────────────┐
│  WELD MONITOR                               [REC ●]         │
├─────────────────────────────────────────────────────────────┤
│  REAL-TIME DATA                                              │
│  ┌─────────────────────────────────────────────────────────┐│
│  │ Current:    ████████████████░░░░  185 A / 250 A         ││
│  │ Voltage:    ██████████████░░░░░░  22.5 V / 30 V         ││
│  │ WFS:        ████████████░░░░░░░░  8.2 m/min             ││
│  │ Arc Time:   ██████░░░░░░░░░░░░░░  00:02:34              ││
│  └─────────────────────────────────────────────────────────┘│
├─────────────────────────────────────────────────────────────┤
│  WAVEFORM                                                    │
│  ┌─────────────────────────────────────────────────────────┐│
│  │         ___     ___     ___                              ││
│  │ I(t)   /   \   /   \   /   \                            ││
│  │     __/     \_/     \_/     \__                         ││
│  │                                                          ││
│  │ V(t)  ----____----____----____                          ││
│  └─────────────────────────────────────────────────────────┘│
├─────────────────────────────────────────────────────────────┤
│  STATISTICS                                                  │
│  ┌─────────────────┐  ┌─────────────────┐  ┌──────────────┐│
│  │ Arc Stability   │  │ Wire Used       │  │ Faults       ││
│  │     98.5%       │  │    2.5 m        │  │      0       ││
│  └─────────────────┘  └─────────────────┘  └──────────────┘│
└─────────────────────────────────────────────────────────────┘
```

**DoD:**
- [ ] Real-time graphs (30fps)
- [ ] Data logging to file
- [ ] Statistics calculation

---

## 4. I/O INTERFACE SPECIFICATION

### 4.1 Connector Pinout

**Welding Interface Connector (DB25):**
| Pin | Signal | Direction | Type | Description |
|-----|--------|-----------|------|-------------|
| 1 | DO_GAS_VALVE | OUT | Digital | Gas solenoid valve |
| 2 | DO_ARC_START | OUT | Digital | Arc start command |
| 3 | DO_WIRE_INCH | OUT | Digital | Wire inch (jog) |
| 4 | DO_WIRE_RETRACT | OUT | Digital | Wire retract |
| 5 | DO_SPARE | OUT | Digital | Reserved |
| 6-10 | GND | - | - | Ground |
| 11 | DI_ARC_OK | IN | Digital | Arc established |
| 12 | DI_READY | IN | Digital | Power source ready |
| 13 | DI_WELD_ERROR | IN | Digital | Error signal |
| 14 | DI_WIRE_STUCK | IN | Digital | Wire stuck |
| 15 | DI_GAS_OK | IN | Digital | Gas pressure OK |
| 16-18 | GND | - | - | Ground |
| 19 | AO_WFS_REF | OUT | Analog | Wire feed speed 0-10V |
| 20 | AO_VOLT_REF | OUT | Analog | Voltage reference 0-10V |
| 21-22 | AGND | - | - | Analog ground |
| 23-25 | SHIELD | - | - | Cable shield |

### 4.2 Signal Timing

**Weld Start Sequence Timing:**
```
Time(ms):    0    500   700   900   1100  1300  1500
            │     │     │     │     │     │     │
GAS_VALVE:  ─┐____│_____│_____│_____│_____│_____│_____
             │    │     │     │     │     │     │
ARC_START:  ─────┐│_____│_____│_____│_____│_____│_____
             │   ││     │     │     │     │     │
WIRE_FEED:  ─────┘│─────│─────│─────│─────│─────│─────
             │    │     │     │     │     │     │
ARC_OK:     ──────│─────│┐____│_____│_____│_____│_____
             │    │     ││    │     │     │     │
State:      IDLE  PREFLOW IGNITION  STABILIZE   WELD
                  500ms   200ms      200ms
```

---

## 5. TESTING & DEFINITION OF DONE

### 5.1 Unit Test Coverage

| Module | Test File | Coverage Target |
|--------|-----------|-----------------|
| WeldingIO | test_WeldingIO.cpp | 90% |
| WeldingSequencer | test_WeldingSequencer.cpp | 95% |
| WeldPatternGenerator | test_WeldPatternGenerator.cpp | 90% |
| WeldProgramManager | test_WeldProgramManager.cpp | 85% |

### 5.2 Integration Tests

| Test Case | Description | Pass Criteria |
|-----------|-------------|---------------|
| IT-W-01 | Full weld cycle | Complete IDLE→WELD→IDLE without errors |
| IT-W-02 | Ignition failure | Retry 2 times, then FAULT |
| IT-W-03 | Emergency stop | Go to POSTFLOW within 100ms |
| IT-W-04 | Weaving + MOVL | Weaving superimposed correctly |
| IT-W-05 | Program execution | All instructions execute in order |

### 5.3 Hardware-in-Loop Tests

| Test Case | Description | Equipment |
|-----------|-------------|-----------|
| HIL-W-01 | I/O timing | Oscilloscope + simulated power source |
| HIL-W-02 | Arc simulation | Arc simulator box |
| HIL-W-03 | Full weld | Real power source (no wire) |

### 5.4 Phase 3 Acceptance Criteria

- [ ] Robot performs 100mm straight weld with weaving
- [ ] Gas pre-flow, arc ignition, weld, crater fill, post-flow sequence correct
- [ ] UI displays real-time current, voltage, arc status
- [ ] Ignition failure and wire stuck handled correctly
- [ ] Weld program saved and loaded successfully
- [ ] All unit tests pass (>90% coverage)
- [ ] Integration tests pass
- [ ] Documentation complete

---

## 6. DEPENDENCIES & PREREQUISITES

### 6.1 Phase 2 Prerequisites
Trước khi bắt đầu Phase 3, các module sau phải hoàn thành:
- [P2-01] SystemStateManager FSM
- [P2-05] KinematicsEngine (FK/IK)
- [P2-08] TrajectoryPlanner (Ruckig OTG)
- [P2-12] grblHAL Driver

### 6.2 Hardware Requirements
- Welding I/O board (5 DO, 5 DI, 2 AO)
- Welding power source với interface chuẩn
- Gas solenoid valve (24V DC)
- Arc simulator (cho testing)

### 6.3 Software Dependencies
- Eigen3 (đã có từ Phase 2)
- JSON library (nlohmann/json)
- ZeroMQ (đã có từ Phase 1)

---

## 7. REVISION HISTORY

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 01/02/2026 | - | Initial draft |

---

*Document generated following project roadmap structure.*
