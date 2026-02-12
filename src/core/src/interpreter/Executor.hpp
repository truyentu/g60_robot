#pragma once

/**
 * @file Executor.hpp
 * @brief Program executor for RPL
 *
 * Part of Phase 8: Virtual Simulation (IMPL_P8_02)
 */

#include "AST.hpp"
#include <functional>
#include <unordered_map>
#include <atomic>
#include <array>
#include <mutex>
#include <condition_variable>

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

    // I/O
    void setInput(int index, bool value);
    bool getInput(int index) const;
    void setOutput(int index, bool value);
    bool getOutput(int index) const;

    // Error
    std::string getLastError() const { return m_lastError; }

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

    // Block Selection / Undo
    std::vector<ExecutionSnapshot> m_undoStack;
    static constexpr size_t MAX_UNDO_SIZE = 1000;
    std::unordered_map<int, size_t> m_lineToPC;  // sourceLine → body index
    std::vector<double> m_currentPosition;        // Current robot TCP for undo

    // Callbacks
    MotionCallback m_motionCallback;
    LineCallback m_lineCallback;
    WaitCallback m_waitCallback;
    IOCallback m_outputCallback;

    // Execution
    void executeStatements(const std::vector<StmtPtr>& statements);
    void executeStatement(const StmtPtr& stmt);
    double evaluateExpression(const ExprPtr& expr);
    bool evaluateCondition(const ExprPtr& expr);

    void checkPauseOrStop();
    void setError(const std::string& msg);
    std::vector<double> parseRobtargetValues(const std::string& raw);
    void pushUndoSnapshot(const StmtPtr& stmt);
    static int getSourceLine(const StmtPtr& stmt);
};

} // namespace interpreter
} // namespace robot_controller
