#pragma once

/**
 * @file Executor.hpp
 * @brief Program executor for KRL (KUKA Robot Language)
 *
 * Strict KUKA KRL syntax based on KSS 8.x specifications.
 */

#include "AST.hpp"
#include <functional>
#include <unordered_map>
#include <atomic>
#include <array>
#include <mutex>
#include <condition_variable>
#include <stdexcept>

namespace robot_controller {
namespace interpreter {

enum class ExecutionState {
    IDLE,
    RUNNING,
    PAUSED,
    STEPPING,
    STOPPED,
    COMPLETED,
    ERROR
};

inline std::string executionStateToString(ExecutionState state) {
    switch (state) {
        case ExecutionState::IDLE: return "IDLE";
        case ExecutionState::RUNNING: return "RUNNING";
        case ExecutionState::PAUSED: return "PAUSED";
        case ExecutionState::STEPPING: return "STEPPING";
        case ExecutionState::STOPPED: return "STOPPED";
        case ExecutionState::COMPLETED: return "COMPLETED";
        case ExecutionState::ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

// KRL control flow exceptions (used internally by Executor)
struct ExitLoopException : std::exception {};
struct ContinueLoopException : std::exception {};

struct GotoException : std::exception {
    std::string label;
    explicit GotoException(const std::string& l) : label(l) {}
};

/**
 * Snapshot of executor state before executing a statement.
 * Used by the undo stack for BWD (backward step) support.
 */
struct ExecutionSnapshot {
    size_t pc = 0;
    int sourceLine = 0;

    // State before execution (restored on BWD)
    std::unordered_map<std::string, double> variables;
    std::array<bool, 128> outputs{};

    // Motion reversal info
    bool wasMotion = false;
    std::string motionType;
    std::vector<double> prevPosition;  // Robot TCP before this motion
};

class Executor {
public:
    using MotionCallback = std::function<void(const MotionStmt&)>;
    using LineCallback = std::function<void(int line)>;
    using WaitCallback = std::function<void(double seconds)>;
    using IOCallback = std::function<void(int index, bool value)>;
    using BaseChangeCallback = std::function<void(int baseIndex)>;

    Executor();
    ~Executor();

    // Load program
    void loadProgram(const ProgramStmt& program);

    // Execution control
    void run();
    void step();      // Execute one statement
    void pause();
    void resume();
    void stop();
    void reset();

    // Block Selection (KUKA-style Satzanwahl)
    void blockSelect(int sourceLine);
    void executeLine(int sourceLine);
    void executeRange(int fromLine, int toLine);
    void runFromLine(int sourceLine);
    void backward();

    // State
    ExecutionState getState() const { return m_state.load(); }
    int getCurrentLine() const { return m_currentLine; }
    size_t getCurrentPC() const { return m_pc; }
    std::string getProgramName() const { return m_program.name; }
    bool canBackward() const;
    int getSourceLineForPC(size_t pc) const;

    // Position tracking (called after motion completes to keep undo accurate)
    void updateCurrentPosition(const std::vector<double>& pos);

    // Callbacks
    void setMotionCallback(MotionCallback cb) { m_motionCallback = cb; }
    void setLineCallback(LineCallback cb) { m_lineCallback = cb; }
    void setWaitCallback(WaitCallback cb) { m_waitCallback = cb; }
    void setOutputCallback(IOCallback cb) { m_outputCallback = cb; }
    void setBaseChangeCallback(BaseChangeCallback cb) { m_baseChangeCallback = std::move(cb); }

    // Point database
    void setPoint(const std::string& name, const std::vector<double>& values);
    std::vector<double> getPoint(const std::string& name) const;
    const std::unordered_map<std::string, std::vector<double>>& getPoints() const { return m_points; }
    void clearPoints() { m_points.clear(); }

    // Variables
    void setVariable(const std::string& name, double value);
    double getVariable(const std::string& name) const;
    const std::unordered_map<std::string, double>& getVariables() const { return m_variables; }
    void clearVariables() { m_variables.clear(); }

    // System variables (KRL dotted notation: $VEL.CP, $APO.CDIS, etc.)
    struct SystemVariables {
        double ovPro = 100.0;              // $OV_PRO (0-100%)

        // Path velocity (Cartesian)
        double velCp = 2.0;                // $VEL.CP (m/s, default 2.0)
        std::array<double, 6> velAxis;     // $VEL_AXIS[1..6] (%, default 100)

        // Acceleration
        double accCp = 1.0;                // $ACC.CP (m/s^2)
        std::array<double, 6> accAxis;     // $ACC_AXIS[1..6] (%, default 100)

        // Approximation
        double apoCdis = 0.0;              // $APO.CDIS (mm)
        double apoCptp = 0.0;              // $APO.CPTP (%)
        double apoCori = 0.0;              // $APO.CORI (deg)
        double apoCvel = 0.0;              // $APO.CVEL (%)

        // Tool and Base (indices)
        int toolIndex = 1;                 // $TOOL = TOOL_DATA[N]
        int baseIndex = 0;                 // $BASE = BASE_DATA[N]

        // Orientation control for CIRC
        int oriType = 0;                   // $ORI_TYPE: 0=#VAR, 1=#CONSTANT, 2=#JOINT
        int circType = 0;                  // $CIRC_TYPE: 0=#BASE, 1=#PATH

        SystemVariables() {
            velAxis.fill(100.0);
            accAxis.fill(100.0);
        }
    };

    double getSystemVariable(const std::string& name, const std::string& subField = "") const;
    void setSystemVariable(const std::string& name, const std::string& subField, double value);
    const SystemVariables& getSystemVars() const { return m_sysVars; }

    // I/O
    void setInput(int index, bool value);
    bool getInput(int index) const;
    void setOutput(int index, bool value);
    bool getOutput(int index) const;

    // Error
    std::string getLastError() const { return m_lastError; }

    // Configuration
    void setMaxLoopIterations(size_t max) { m_maxLoopIterations = max; }

    // Expression evaluation (public for motion callback to evaluate aggregate fields)
    double evaluateExpression(const ExprPtr& expr);

private:
    ProgramStmt m_program;
    std::atomic<ExecutionState> m_state{ExecutionState::IDLE};
    size_t m_pc = 0;  // Program counter
    int m_currentLine = 0;
    std::string m_lastError;

    // Synchronization
    mutable std::mutex m_mutex;
    std::condition_variable m_pauseCV;
    bool m_stepRequested = false;

    // Data
    std::unordered_map<std::string, std::vector<double>> m_points;
    std::unordered_map<std::string, double> m_variables;
    std::array<bool, 128> m_inputs{};
    std::array<bool, 128> m_outputs{};

    // KRL System Variables instance
    SystemVariables m_sysVars;

    // Block Selection / Undo
    std::vector<ExecutionSnapshot> m_undoStack;
    static constexpr size_t MAX_UNDO_SIZE = 1000;
    std::unordered_map<int, size_t> m_lineToPC;     // sourceLine → body index
    std::unordered_map<std::string, size_t> m_labelToPC;  // label → body index (GOTO)
    std::vector<double> m_currentPosition;            // Current robot TCP for undo

    // Safety
    size_t m_maxLoopIterations = 1000000;

    // Callbacks
    MotionCallback m_motionCallback;
    LineCallback m_lineCallback;
    WaitCallback m_waitCallback;
    IOCallback m_outputCallback;
    BaseChangeCallback m_baseChangeCallback;

    // Execution
    void executeStatements(const std::vector<StmtPtr>& statements);
    void executeStatement(const StmtPtr& stmt);
    bool evaluateCondition(const ExprPtr& expr);

    void checkPauseOrStop();
    void setError(const std::string& msg);
    std::vector<double> parseRobtargetValues(const std::string& raw);
    void pushUndoSnapshot(const StmtPtr& stmt);
    static int getSourceLine(const StmtPtr& stmt);
    void buildLabelMap(const std::vector<StmtPtr>& statements);
};

} // namespace interpreter
} // namespace robot_controller
