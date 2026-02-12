/**
 * @file Executor.cpp
 * @brief Executor implementation for RPL
 *
 * Part of Phase 8: Virtual Simulation (IMPL_P8_02)
 */

#include "Executor.hpp"
#include "../logging/Logger.hpp"
#include <thread>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <sstream>

namespace robot_controller {
namespace interpreter {

Executor::Executor() {
    // Initialize default HOME position
    m_points["HOME"] = {0, 0, 0, 0, 0, 0};
}

Executor::~Executor() {
    stop();
}

void Executor::loadProgram(const ProgramStmt& program) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_program = program;
    m_pc = 0;
    m_currentLine = 0;
    m_state = ExecutionState::IDLE;
    m_lastError.clear();
    m_undoStack.clear();
    m_currentPosition.clear();

    // Build sourceLine → PC lookup table
    m_lineToPC.clear();
    for (size_t i = 0; i < m_program.body.size(); ++i) {
        int line = getSourceLine(m_program.body[i]);
        if (line > 0) m_lineToPC[line] = i;
    }

    // Process CONST declarations - extract point data
    for (const auto& constDecl : program.constDecls) {
        if (!constDecl) continue;
        std::visit([this](auto&& s) {
            using T = std::decay_t<decltype(s)>;
            if constexpr (std::is_same_v<T, ConstDeclStmt>) {
                // Parse robtarget raw value: [[x,y,z],[q1,q2,q3,q4],[cf1,cf4,cf6,cfx],[e1..e6]]
                if (s.type == "robtarget" && !s.rawValue.empty()) {
                    auto values = parseRobtargetValues(s.rawValue);
                    if (!values.empty()) {
                        m_points[s.name] = values;
                        LOG_DEBUG("CONST robtarget {} = [{}, {}, {}, ...]", s.name,
                            values.size() > 0 ? values[0] : 0,
                            values.size() > 1 ? values[1] : 0,
                            values.size() > 2 ? values[2] : 0);
                    }
                }
            }
        }, *constDecl);
    }

    LOG_INFO("Loaded program: {} ({} consts, {} statements, {} line mappings)",
        program.name, program.constDecls.size(), program.body.size(), m_lineToPC.size());
}

void Executor::run() {
    if (m_program.body.empty()) {
        LOG_WARN("No program loaded");
        return;
    }

    m_state = ExecutionState::RUNNING;
    LOG_INFO("Running program: {}", m_program.name);

    try {
        executeStatements(m_program.body);

        if (m_state == ExecutionState::RUNNING) {
            m_state = ExecutionState::COMPLETED;
            LOG_INFO("Program completed: {}", m_program.name);
        }
    } catch (const std::exception& e) {
        setError(e.what());
    }
}

void Executor::step() {
    if (m_program.body.empty()) {
        LOG_WARN("No program loaded");
        return;
    }

    if (m_state == ExecutionState::COMPLETED || m_state == ExecutionState::ERROR) {
        return;
    }

    m_state = ExecutionState::STEPPING;
    m_stepRequested = true;

    if (m_pc < m_program.body.size()) {
        try {
            executeStatement(m_program.body[m_pc]);
            m_pc++;

            if (m_pc >= m_program.body.size()) {
                m_state = ExecutionState::COMPLETED;
                LOG_INFO("Program completed: {}", m_program.name);
            } else {
                m_state = ExecutionState::PAUSED;
            }
        } catch (const std::exception& e) {
            setError(e.what());
        }
    }

    m_stepRequested = false;
}

void Executor::pause() {
    if (m_state == ExecutionState::RUNNING) {
        m_state = ExecutionState::PAUSED;
        LOG_INFO("Program paused");
    }
}

void Executor::resume() {
    if (m_state == ExecutionState::PAUSED) {
        m_state = ExecutionState::RUNNING;
        m_pauseCV.notify_all();
        LOG_INFO("Program resumed");
    }
}

void Executor::stop() {
    ExecutionState expected = m_state.load();
    if (expected == ExecutionState::RUNNING || expected == ExecutionState::PAUSED) {
        m_state = ExecutionState::STOPPED;
        m_pauseCV.notify_all();
        LOG_INFO("Program stopped");
    }
}

void Executor::reset() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_pc = 0;
    m_currentLine = 0;
    m_state = ExecutionState::IDLE;
    m_lastError.clear();
    m_undoStack.clear();
    m_currentPosition.clear();
    LOG_INFO("Program reset");
}

void Executor::setPoint(const std::string& name, const std::vector<double>& values) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_points[name] = values;
}

std::vector<double> Executor::getPoint(const std::string& name) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    auto it = m_points.find(name);
    if (it != m_points.end()) {
        return it->second;
    }
    return {};
}

void Executor::setVariable(const std::string& name, double value) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_variables[name] = value;
}

double Executor::getVariable(const std::string& name) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    auto it = m_variables.find(name);
    if (it != m_variables.end()) {
        return it->second;
    }
    return 0.0;
}

void Executor::setInput(int index, bool value) {
    if (index >= 0 && index < 128) {
        m_inputs[index] = value;
    }
}

bool Executor::getInput(int index) const {
    if (index >= 0 && index < 128) {
        return m_inputs[index];
    }
    return false;
}

void Executor::setOutput(int index, bool value) {
    if (index >= 0 && index < 128) {
        m_outputs[index] = value;
        if (m_outputCallback) {
            m_outputCallback(index, value);
        }
    }
}

bool Executor::getOutput(int index) const {
    if (index >= 0 && index < 128) {
        return m_outputs[index];
    }
    return false;
}

void Executor::checkPauseOrStop() {
    if (m_state == ExecutionState::STOPPED) {
        throw std::runtime_error("Program stopped");
    }

    if (m_state == ExecutionState::PAUSED) {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_pauseCV.wait(lock, [this] {
            return m_state != ExecutionState::PAUSED;
        });

        if (m_state == ExecutionState::STOPPED) {
            throw std::runtime_error("Program stopped");
        }
    }
}

void Executor::setError(const std::string& msg) {
    m_state = ExecutionState::ERROR;
    m_lastError = msg;
    LOG_ERROR("Execution error: {}", msg);
}

void Executor::executeStatements(const std::vector<StmtPtr>& statements) {
    for (size_t i = 0; i < statements.size(); ++i) {
        checkPauseOrStop();

        m_pc = i;
        executeStatement(statements[i]);
    }
}

void Executor::executeStatement(const StmtPtr& stmt) {
    if (!stmt) return;

    // Push undo snapshot BEFORE executing (for BWD support)
    pushUndoSnapshot(stmt);

    std::visit([this](auto&& s) {
        using T = std::decay_t<decltype(s)>;

        if constexpr (std::is_same_v<T, DeclareStmt>) {
            // DECL REAL x = 10
            double value = 0.0;
            if (s.initializer) {
                value = evaluateExpression(s.initializer);
            }
            setVariable(s.name, value);
            LOG_DEBUG("Declared {} {} = {}", s.type, s.name, value);
        }
        else if constexpr (std::is_same_v<T, ConstDeclStmt>) {
            // CONST robtarget p1 := [[...]]
            // Already processed during loadProgram, but handle if inside body
            if (s.type == "robtarget" && !s.rawValue.empty()) {
                auto values = parseRobtargetValues(s.rawValue);
                if (!values.empty()) {
                    m_points[s.name] = values;
                }
            }
            LOG_DEBUG("CONST {} {}", s.type, s.name);
        }
        else if constexpr (std::is_same_v<T, FunctionCallStmt>) {
            // ArcStart(...), ArcEnd, etc.
            m_currentLine = s.sourceLine;
            if (m_lineCallback) {
                m_lineCallback(s.sourceLine);
            }
            LOG_INFO("Function call: {} ({} args)", s.name, s.args.size());
            // Stub: simulate arc start/end with a small delay
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        else if constexpr (std::is_same_v<T, AssignStmt>) {
            // x = 10
            double value = evaluateExpression(s.value);
            setVariable(s.name, value);
            LOG_DEBUG("Assigned {} = {}", s.name, value);
        }
        else if constexpr (std::is_same_v<T, SystemAssignStmt>) {
            // $OUT[1] = TRUE
            int index = static_cast<int>(evaluateExpression(s.index));
            bool value = evaluateCondition(s.value);

            std::string varName = s.name;
            std::transform(varName.begin(), varName.end(), varName.begin(), ::toupper);

            if (varName == "OUT") {
                setOutput(index, value);
                LOG_DEBUG("$OUT[{}] = {}", index, value);
            }
        }
        else if constexpr (std::is_same_v<T, MotionStmt>) {
            m_currentLine = s.sourceLine;
            if (m_lineCallback) {
                m_lineCallback(s.sourceLine);
            }

            LOG_DEBUG("Motion {} at line {}", s.type, s.sourceLine);

            if (m_motionCallback) {
                m_motionCallback(s);
            }
        }
        else if constexpr (std::is_same_v<T, WaitStmt>) {
            m_currentLine = s.sourceLine;
            if (m_lineCallback) {
                m_lineCallback(s.sourceLine);
            }

            if (s.type == WaitStmt::WaitType::TIME) {
                double seconds = evaluateExpression(s.value);
                LOG_DEBUG("WAIT SEC {}", seconds);

                if (m_waitCallback) {
                    m_waitCallback(seconds);
                } else {
                    // Default: sleep
                    std::this_thread::sleep_for(
                        std::chrono::milliseconds(static_cast<int>(seconds * 1000)));
                }
            } else {
                // WAIT FOR condition
                LOG_DEBUG("WAIT FOR condition");
                while (!evaluateCondition(s.value)) {
                    checkPauseOrStop();
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
            }
        }
        else if constexpr (std::is_same_v<T, IfStmt>) {
            m_currentLine = s.sourceLine;
            if (m_lineCallback) {
                m_lineCallback(s.sourceLine);
            }

            if (evaluateCondition(s.condition)) {
                executeStatements(s.thenBranch);
            } else if (!s.elseBranch.empty()) {
                executeStatements(s.elseBranch);
            }
        }
        else if constexpr (std::is_same_v<T, LoopStmt>) {
            m_currentLine = s.sourceLine;
            if (m_lineCallback) {
                m_lineCallback(s.sourceLine);
            }

            int count = s.count ? static_cast<int>(evaluateExpression(s.count)) : -1;
            int iteration = 0;

            while (count < 0 || iteration < count) {
                checkPauseOrStop();
                executeStatements(s.body);
                iteration++;
            }
        }
        else if constexpr (std::is_same_v<T, WhileStmt>) {
            m_currentLine = s.sourceLine;
            if (m_lineCallback) {
                m_lineCallback(s.sourceLine);
            }

            while (evaluateCondition(s.condition)) {
                checkPauseOrStop();
                executeStatements(s.body);
            }
        }
        else if constexpr (std::is_same_v<T, ProgramStmt>) {
            // Should not happen at statement level
        }
    }, *stmt);
}

double Executor::evaluateExpression(const ExprPtr& expr) {
    if (!expr) return 0.0;

    return std::visit([this](auto&& e) -> double {
        using T = std::decay_t<decltype(e)>;

        if constexpr (std::is_same_v<T, LiteralExpr>) {
            return std::visit([](auto&& v) -> double {
                using V = std::decay_t<decltype(v)>;
                if constexpr (std::is_same_v<V, double>) return v;
                else if constexpr (std::is_same_v<V, int>) return static_cast<double>(v);
                else if constexpr (std::is_same_v<V, bool>) return v ? 1.0 : 0.0;
                else return 0.0;
            }, e.value);
        }
        else if constexpr (std::is_same_v<T, VariableExpr>) {
            return getVariable(e.name);
        }
        else if constexpr (std::is_same_v<T, SystemVarExpr>) {
            int index = e.index ? static_cast<int>(evaluateExpression(e.index)) : 0;

            std::string varName = e.name;
            std::transform(varName.begin(), varName.end(), varName.begin(), ::toupper);

            if (varName == "IN") {
                return getInput(index) ? 1.0 : 0.0;
            } else if (varName == "OUT") {
                return getOutput(index) ? 1.0 : 0.0;
            }
            return 0.0;
        }
        else if constexpr (std::is_same_v<T, BinaryExpr>) {
            double left = evaluateExpression(e.left);
            double right = evaluateExpression(e.right);

            if (e.op == "+") return left + right;
            if (e.op == "-") return left - right;
            if (e.op == "*") return left * right;
            if (e.op == "/") return right != 0 ? left / right : 0;
            if (e.op == "==") return (std::abs(left - right) < 1e-9) ? 1.0 : 0.0;
            if (e.op == "<>") return (std::abs(left - right) >= 1e-9) ? 1.0 : 0.0;
            if (e.op == "<") return left < right ? 1.0 : 0.0;
            if (e.op == ">") return left > right ? 1.0 : 0.0;
            if (e.op == "<=") return left <= right ? 1.0 : 0.0;
            if (e.op == ">=") return left >= right ? 1.0 : 0.0;
            if (e.op == "AND") return (left != 0 && right != 0) ? 1.0 : 0.0;
            if (e.op == "OR") return (left != 0 || right != 0) ? 1.0 : 0.0;

            return 0.0;
        }
        else if constexpr (std::is_same_v<T, UnaryExpr>) {
            double operand = evaluateExpression(e.operand);

            if (e.op == "-") return -operand;
            if (e.op == "NOT") return operand == 0 ? 1.0 : 0.0;

            return operand;
        }
        else if constexpr (std::is_same_v<T, PointExpr>) {
            // Points don't evaluate to a single number
            return 0.0;
        }

        return 0.0;
    }, *expr);
}

bool Executor::evaluateCondition(const ExprPtr& expr) {
    return evaluateExpression(expr) != 0.0;
}

// ============================================================================
// Block Selection Infrastructure
// ============================================================================

int Executor::getSourceLine(const StmtPtr& stmt) {
    if (!stmt) return 0;
    return std::visit([](auto&& s) -> int {
        using T = std::decay_t<decltype(s)>;
        if constexpr (std::is_same_v<T, MotionStmt>) return s.sourceLine;
        else if constexpr (std::is_same_v<T, WaitStmt>) return s.sourceLine;
        else if constexpr (std::is_same_v<T, IfStmt>) return s.sourceLine;
        else if constexpr (std::is_same_v<T, LoopStmt>) return s.sourceLine;
        else if constexpr (std::is_same_v<T, WhileStmt>) return s.sourceLine;
        else if constexpr (std::is_same_v<T, FunctionCallStmt>) return s.sourceLine;
        else if constexpr (std::is_same_v<T, ConstDeclStmt>) return s.sourceLine;
        else return 0;
    }, *stmt);
}

void Executor::pushUndoSnapshot(const StmtPtr& stmt) {
    ExecutionSnapshot snap;
    snap.pc = m_pc;
    snap.sourceLine = getSourceLine(stmt);
    snap.variables = m_variables;
    snap.outputs = m_outputs;
    snap.prevPosition = m_currentPosition;

    // Check if this is a motion statement
    if (auto* ms = std::get_if<MotionStmt>(stmt.get())) {
        snap.wasMotion = true;
        snap.motionType = ms->type;
    }

    // FIFO eviction when stack is full
    if (m_undoStack.size() >= MAX_UNDO_SIZE) {
        m_undoStack.erase(m_undoStack.begin());
    }
    m_undoStack.push_back(std::move(snap));
}

bool Executor::canBackward() const {
    return !m_undoStack.empty();
}

void Executor::updateCurrentPosition(const std::vector<double>& pos) {
    m_currentPosition = pos;
}

int Executor::getSourceLineForPC(size_t pc) const {
    if (pc < m_program.body.size()) {
        return getSourceLine(m_program.body[pc]);
    }
    return 0;
}

void Executor::blockSelect(int sourceLine) {
    auto it = m_lineToPC.find(sourceLine);
    if (it == m_lineToPC.end()) {
        LOG_WARN("Block Select: line {} not found in lineToPC map", sourceLine);
        return;
    }

    size_t targetPC = it->second;

    // Pre-scan: resolve DeclareStmt before targetPC
    for (size_t i = 0; i < targetPC && i < m_program.body.size(); ++i) {
        if (!m_program.body[i]) continue;
        if (auto* decl = std::get_if<DeclareStmt>(m_program.body[i].get())) {
            double val = decl->initializer ? evaluateExpression(decl->initializer) : 0.0;
            m_variables[decl->name] = val;
        }
    }

    // Set PC, clear undo (no history before block select)
    m_pc = targetPC;
    m_currentLine = sourceLine;
    m_undoStack.clear();
    m_state = ExecutionState::PAUSED;

    if (m_lineCallback) {
        m_lineCallback(sourceLine);
    }

    LOG_INFO("Block Select: jumped to line {}, pc={}", sourceLine, targetPC);
}

void Executor::executeLine(int sourceLine) {
    blockSelect(sourceLine);

    if (m_pc < m_program.body.size()) {
        try {
            m_state = ExecutionState::STEPPING;
            executeStatement(m_program.body[m_pc]);
            m_pc++;
            m_state = ExecutionState::PAUSED;
        } catch (const std::exception& e) {
            setError(e.what());
        }
    }
}

void Executor::executeRange(int fromLine, int toLine) {
    auto fromIt = m_lineToPC.find(fromLine);
    auto toIt = m_lineToPC.find(toLine);

    if (fromIt == m_lineToPC.end() || toIt == m_lineToPC.end()) {
        LOG_WARN("Execute Range: line {} or {} not found", fromLine, toLine);
        return;
    }

    size_t fromPC = fromIt->second;
    size_t toPC = toIt->second;

    // Resolve deps up to fromPC
    blockSelect(fromLine);

    m_state = ExecutionState::RUNNING;
    try {
        for (m_pc = fromPC; m_pc <= toPC && m_pc < m_program.body.size(); ++m_pc) {
            checkPauseOrStop();
            executeStatement(m_program.body[m_pc]);
        }
        m_state = ExecutionState::PAUSED;
        LOG_INFO("Execute Range: lines {} to {} completed", fromLine, toLine);
    } catch (const std::exception& e) {
        if (m_state != ExecutionState::STOPPED) {
            setError(e.what());
        }
    }
}

void Executor::runFromLine(int sourceLine) {
    blockSelect(sourceLine);
    m_state = ExecutionState::RUNNING;

    try {
        for (; m_pc < m_program.body.size(); ++m_pc) {
            checkPauseOrStop();
            executeStatement(m_program.body[m_pc]);
        }

        if (m_state == ExecutionState::RUNNING) {
            m_state = ExecutionState::COMPLETED;
            LOG_INFO("Program completed (from line {})", sourceLine);
        }
    } catch (const std::exception& e) {
        if (m_state != ExecutionState::STOPPED) {
            setError(e.what());
        }
    }
}

void Executor::backward() {
    if (m_undoStack.empty()) {
        LOG_WARN("BWD: undo stack empty");
        return;
    }

    auto snap = std::move(m_undoStack.back());
    m_undoStack.pop_back();

    // Restore variables
    m_variables = std::move(snap.variables);

    // Restore I/O and fire callbacks for changes
    for (int i = 0; i < 128; ++i) {
        if (m_outputs[i] != snap.outputs[i]) {
            m_outputs[i] = snap.outputs[i];
            if (m_outputCallback) {
                m_outputCallback(i, snap.outputs[i]);
            }
        }
    }

    // Reverse motion if applicable
    if (snap.wasMotion && !snap.prevPosition.empty()) {
        MotionStmt reverseMotion;
        reverseMotion.type = snap.motionType;
        reverseMotion.sourceLine = snap.sourceLine;
        reverseMotion.target = makeExpr(PointExpr{"_BWD_TARGET", snap.prevPosition, false});

        if (m_motionCallback) {
            m_motionCallback(reverseMotion);
        }
        m_currentPosition = snap.prevPosition;
    }

    // Restore PC
    m_pc = snap.pc;
    m_currentLine = snap.sourceLine;
    m_state = ExecutionState::PAUSED;

    if (m_lineCallback) {
        m_lineCallback(snap.sourceLine);
    }

    LOG_INFO("BWD: reversed to line {}, pc={}", snap.sourceLine, snap.pc);
}

// ============================================================================
// Robtarget Parser
// ============================================================================

std::vector<double> Executor::parseRobtargetValues(const std::string& raw) {
    // Parse [[x,y,z],[q1,q2,q3,q4],[cf1,cf4,cf6,cfx],[e1..e6]]
    // Extract just the position values [x,y,z] for now (first inner bracket)
    std::vector<double> result;

    // Find all numbers in the raw string
    std::vector<double> allNumbers;
    std::string current;
    bool inNumber = false;

    for (size_t i = 0; i < raw.size(); ++i) {
        char c = raw[i];
        if (std::isdigit(c) || c == '.' || c == '-' ||
            c == 'E' || c == 'e' || (c == '+' && i > 0 && (raw[i-1] == 'E' || raw[i-1] == 'e'))) {
            current += c;
            inNumber = true;
        } else {
            if (inNumber && !current.empty()) {
                try {
                    allNumbers.push_back(std::stod(current));
                } catch (...) {}
                current.clear();
                inNumber = false;
            }
        }
    }
    if (!current.empty()) {
        try {
            allNumbers.push_back(std::stod(current));
        } catch (...) {}
    }

    // robtarget has: [x,y,z] [q1,q2,q3,q4] [cf1,cf4,cf6,cfx] [e1..e6]
    // We need at least position (first 3 values) for point database
    // Store all for full fidelity: x,y,z,q1,q2,q3,q4,...
    if (allNumbers.size() >= 3) {
        // Return first 3 as x,y,z for motion target
        // And next 4 as quaternion orientation if available
        result = allNumbers;
    }

    return result;
}

} // namespace interpreter
} // namespace robot_controller
