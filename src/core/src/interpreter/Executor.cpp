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
    LOG_INFO("Loaded program: {}", program.name);
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

} // namespace interpreter
} // namespace robot_controller
