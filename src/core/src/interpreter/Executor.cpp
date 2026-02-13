/**
 * @file Executor.cpp
 * @brief Executor implementation for KRL (KUKA Robot Language)
 *
 * Strict KUKA KRL syntax based on KSS 8.x specifications.
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

    // Build label → PC lookup table (for GOTO)
    buildLabelMap(m_program.body);

    // Process CONST declarations - extract point data
    for (const auto& constDecl : program.constDecls) {
        if (!constDecl) continue;
        std::visit([this](auto&& s) {
            using T = std::decay_t<decltype(s)>;
            if constexpr (std::is_same_v<T, ConstDeclStmt>) {
                std::string typeUpper = s.type;
                std::transform(typeUpper.begin(), typeUpper.end(), typeUpper.begin(), ::toupper);
                if ((typeUpper == "E6POS" || typeUpper == "POS" || typeUpper == "FRAME" ||
                     typeUpper == "E6AXIS" || typeUpper == "AXIS") && !s.rawValue.empty()) {
                    auto values = parseRobtargetValues(s.rawValue);
                    if (!values.empty()) {
                        m_points[s.name] = values;
                        LOG_DEBUG("CONST {} {} = [{}, {}, {}, ...]", s.type, s.name,
                            values.size() > 0 ? values[0] : 0,
                            values.size() > 1 ? values[1] : 0,
                            values.size() > 2 ? values[2] : 0);
                    }
                }
            }
        }, *constDecl);
    }

    LOG_INFO("Loaded program: {} ({} consts, {} statements, {} labels)",
        program.name, program.constDecls.size(), program.body.size(), m_labelToPC.size());
}

void Executor::buildLabelMap(const std::vector<StmtPtr>& statements) {
    m_labelToPC.clear();
    for (size_t i = 0; i < statements.size(); ++i) {
        if (!statements[i]) continue;
        if (auto* label = std::get_if<LabelStmt>(statements[i].get())) {
            m_labelToPC[label->name] = i;
            LOG_DEBUG("Label '{}' at PC={}", label->name, i);
        }
    }
}

// ============================================================================
// Execution Control
// ============================================================================

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
    } catch (const ExitLoopException&) {
        setError("EXIT outside of loop");
    } catch (const ContinueLoopException&) {
        setError("CONTINUE outside of loop");
    } catch (const GotoException& g) {
        setError("GOTO label '" + g.label + "' not found in current scope");
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
    m_sysVars = SystemVariables{};
    LOG_INFO("Program reset");
}

// ============================================================================
// Data Access
// ============================================================================

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

double Executor::getSystemVariable(const std::string& name, const std::string& subField) const {
    std::string upper = name;
    std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);
    std::string upperSub = subField;
    std::transform(upperSub.begin(), upperSub.end(), upperSub.begin(), ::toupper);

    if (upper == "OV_PRO") return m_sysVars.ovPro;

    // $VEL.CP or legacy $VEL (backwards compat)
    if (upper == "VEL") {
        if (upperSub == "CP") return m_sysVars.velCp;
        if (upperSub.empty()) return m_sysVars.velCp;  // Legacy: $VEL alone maps to $VEL.CP
        return 0.0;
    }

    // $ACC.CP or legacy $ACC
    if (upper == "ACC") {
        if (upperSub == "CP") return m_sysVars.accCp;
        if (upperSub.empty()) return m_sysVars.accCp;
        return 0.0;
    }

    // $APO.CDIS, $APO.CPTP, $APO.CORI, $APO.CVEL
    if (upper == "APO") {
        if (upperSub == "CDIS") return m_sysVars.apoCdis;
        if (upperSub == "CPTP") return m_sysVars.apoCptp;
        if (upperSub == "CORI") return m_sysVars.apoCori;
        if (upperSub == "CVEL") return m_sysVars.apoCvel;
        return 0.0;
    }

    // $TOOL (index)
    if (upper == "TOOL") return static_cast<double>(m_sysVars.toolIndex);

    // $BASE (index)
    if (upper == "BASE") return static_cast<double>(m_sysVars.baseIndex);

    // Position fields
    if (upper == "POS_ACT") {
        return !m_currentPosition.empty() ? m_currentPosition[0] : 0.0;
    }

    return 0.0;
}

void Executor::setSystemVariable(const std::string& name, const std::string& subField, double value) {
    std::string upper = name;
    std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);
    std::string upperSub = subField;
    std::transform(upperSub.begin(), upperSub.end(), upperSub.begin(), ::toupper);

    if (upper == "OV_PRO") {
        m_sysVars.ovPro = std::clamp(value, 0.0, 100.0);
        LOG_DEBUG("$OV_PRO = {}", m_sysVars.ovPro);
    } else if (upper == "VEL") {
        if (upperSub == "CP" || upperSub.empty()) {
            m_sysVars.velCp = std::max(0.0, value);
            LOG_DEBUG("$VEL.CP = {} m/s", m_sysVars.velCp);
        }
    } else if (upper == "VEL_AXIS") {
        // Handled via index, not subField - see executeStatement
        LOG_DEBUG("$VEL_AXIS set (no index here)");
    } else if (upper == "ACC") {
        if (upperSub == "CP" || upperSub.empty()) {
            m_sysVars.accCp = std::max(0.0, value);
            LOG_DEBUG("$ACC.CP = {} m/s^2", m_sysVars.accCp);
        }
    } else if (upper == "ACC_AXIS") {
        LOG_DEBUG("$ACC_AXIS set (no index here)");
    } else if (upper == "APO") {
        if (upperSub == "CDIS") {
            m_sysVars.apoCdis = std::max(0.0, value);
            LOG_DEBUG("$APO.CDIS = {} mm", m_sysVars.apoCdis);
        } else if (upperSub == "CPTP") {
            m_sysVars.apoCptp = std::clamp(value, 0.0, 100.0);
            LOG_DEBUG("$APO.CPTP = {} %", m_sysVars.apoCptp);
        } else if (upperSub == "CORI") {
            m_sysVars.apoCori = std::max(0.0, value);
            LOG_DEBUG("$APO.CORI = {} deg", m_sysVars.apoCori);
        } else if (upperSub == "CVEL") {
            m_sysVars.apoCvel = std::clamp(value, 0.0, 100.0);
            LOG_DEBUG("$APO.CVEL = {} %", m_sysVars.apoCvel);
        }
    } else if (upper == "TOOL") {
        m_sysVars.toolIndex = static_cast<int>(value);
        LOG_DEBUG("$TOOL = TOOL_DATA[{}]", m_sysVars.toolIndex);
    } else if (upper == "BASE") {
        m_sysVars.baseIndex = static_cast<int>(value);
        LOG_DEBUG("$BASE = BASE_DATA[{}]", m_sysVars.baseIndex);
    }
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

// ============================================================================
// Internal Execution Helpers
// ============================================================================

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

// ============================================================================
// Statement Execution
// ============================================================================

void Executor::executeStatements(const std::vector<StmtPtr>& statements) {
    size_t i = 0;
    while (i < statements.size()) {
        checkPauseOrStop();
        m_pc = i;

        try {
            executeStatement(statements[i]);
        } catch (const GotoException& g) {
            // Resolve GOTO within this statement block
            auto it = m_labelToPC.find(g.label);
            if (it != m_labelToPC.end()) {
                i = it->second;
                LOG_DEBUG("GOTO '{}' -> PC={}", g.label, i);
                continue;
            }
            // Label not in this block - re-throw to parent
            throw;
        }
        // EXIT and CONTINUE propagate to loop handlers

        ++i;
    }
}

void Executor::executeStatement(const StmtPtr& stmt) {
    if (!stmt) return;

    // Push undo snapshot BEFORE executing (for BWD support)
    pushUndoSnapshot(stmt);

    std::visit([this](auto&& s) {
        using T = std::decay_t<decltype(s)>;

        if constexpr (std::is_same_v<T, DeclareStmt>) {
            double value = 0.0;
            if (s.initializer) {
                value = evaluateExpression(s.initializer);
            }
            setVariable(s.name, value);
            LOG_DEBUG("Declared {} {} = {}", s.type, s.name, value);
        }
        else if constexpr (std::is_same_v<T, ConstDeclStmt>) {
            std::string typeUpper = s.type;
            std::transform(typeUpper.begin(), typeUpper.end(), typeUpper.begin(), ::toupper);
            if ((typeUpper == "E6POS" || typeUpper == "POS" || typeUpper == "FRAME" ||
                 typeUpper == "E6AXIS" || typeUpper == "AXIS") && !s.rawValue.empty()) {
                auto values = parseRobtargetValues(s.rawValue);
                if (!values.empty()) {
                    m_points[s.name] = values;
                }
            }
            LOG_DEBUG("CONST {} {}", s.type, s.name);
        }
        else if constexpr (std::is_same_v<T, FunctionCallStmt>) {
            m_currentLine = s.sourceLine;
            if (m_lineCallback) m_lineCallback(s.sourceLine);
            LOG_INFO("Function call: {} ({} args)", s.name, s.args.size());
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        else if constexpr (std::is_same_v<T, AssignStmt>) {
            double value = evaluateExpression(s.value);
            setVariable(s.name, value);
            LOG_DEBUG("Assigned {} = {}", s.name, value);
        }
        else if constexpr (std::is_same_v<T, SystemAssignStmt>) {
            std::string varName = s.name;
            std::transform(varName.begin(), varName.end(), varName.begin(), ::toupper);

            if (varName == "OUT") {
                int index = s.index ? static_cast<int>(evaluateExpression(s.index)) : 0;
                bool value = evaluateCondition(s.value);
                setOutput(index, value);
                LOG_DEBUG("$OUT[{}] = {}", index, value);
            } else if (varName == "VEL_AXIS" || varName == "ACC_AXIS") {
                // $VEL_AXIS[N] = value or $ACC_AXIS[N] = value
                int index = s.index ? static_cast<int>(evaluateExpression(s.index)) : 1;
                double value = evaluateExpression(s.value);
                if (index >= 1 && index <= 6) {
                    if (varName == "VEL_AXIS") {
                        m_sysVars.velAxis[index - 1] = std::clamp(value, 0.0, 100.0);
                        LOG_DEBUG("$VEL_AXIS[{}] = {} %", index, m_sysVars.velAxis[index - 1]);
                    } else {
                        m_sysVars.accAxis[index - 1] = std::clamp(value, 0.0, 100.0);
                        LOG_DEBUG("$ACC_AXIS[{}] = {} %", index, m_sysVars.accAxis[index - 1]);
                    }
                }
            } else if (varName == "TOOL" || varName == "BASE") {
                // $TOOL = TOOL_DATA[N] or $BASE = BASE_DATA[N]
                // RHS is VariableExpr with name "TOOL_DATA[N]" or "BASE_DATA[N]"
                // Extract index from the composite name
                double value = evaluateExpression(s.value);
                setSystemVariable(varName, s.subField, value);
            } else {
                // $VEL.CP, $ACC.CP, $APO.CDIS, $APO.CPTP, $OV_PRO, etc.
                double value = evaluateExpression(s.value);
                setSystemVariable(varName, s.subField, value);
            }
        }
        else if constexpr (std::is_same_v<T, MotionStmt>) {
            m_currentLine = s.sourceLine;
            if (m_lineCallback) m_lineCallback(s.sourceLine);
            LOG_DEBUG("Motion {} at line {}", s.type, s.sourceLine);
            if (m_motionCallback) m_motionCallback(s);
        }
        else if constexpr (std::is_same_v<T, WaitStmt>) {
            m_currentLine = s.sourceLine;
            if (m_lineCallback) m_lineCallback(s.sourceLine);

            if (s.type == WaitStmt::WaitType::TIME) {
                double seconds = evaluateExpression(s.value);
                LOG_DEBUG("WAIT SEC {}", seconds);
                if (m_waitCallback) {
                    m_waitCallback(seconds);
                } else {
                    std::this_thread::sleep_for(
                        std::chrono::milliseconds(static_cast<int>(seconds * 1000)));
                }
            } else {
                LOG_DEBUG("WAIT FOR condition");
                size_t waitIter = 0;
                while (!evaluateCondition(s.value)) {
                    checkPauseOrStop();
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    if (++waitIter > m_maxLoopIterations) {
                        throw std::runtime_error("WAIT FOR timeout exceeded");
                    }
                }
            }
        }
        else if constexpr (std::is_same_v<T, IfStmt>) {
            m_currentLine = s.sourceLine;
            if (m_lineCallback) m_lineCallback(s.sourceLine);

            if (evaluateCondition(s.condition)) {
                executeStatements(s.thenBranch);
            } else if (!s.elseBranch.empty()) {
                executeStatements(s.elseBranch);
            }
        }
        else if constexpr (std::is_same_v<T, LoopStmt>) {
            m_currentLine = s.sourceLine;
            if (m_lineCallback) m_lineCallback(s.sourceLine);

            size_t iteration = 0;
            while (true) {
                checkPauseOrStop();
                if (++iteration > m_maxLoopIterations) {
                    throw std::runtime_error("LOOP iteration limit exceeded at line " +
                        std::to_string(s.sourceLine));
                }
                try {
                    executeStatements(s.body);
                } catch (const ExitLoopException&) {
                    break;
                } catch (const ContinueLoopException&) {
                    continue;
                }
            }
        }
        else if constexpr (std::is_same_v<T, WhileStmt>) {
            m_currentLine = s.sourceLine;
            if (m_lineCallback) m_lineCallback(s.sourceLine);

            size_t iteration = 0;
            while (evaluateCondition(s.condition)) {
                checkPauseOrStop();
                if (++iteration > m_maxLoopIterations) {
                    throw std::runtime_error("WHILE iteration limit exceeded at line " +
                        std::to_string(s.sourceLine));
                }
                try {
                    executeStatements(s.body);
                } catch (const ExitLoopException&) {
                    break;
                } catch (const ContinueLoopException&) {
                    continue;
                }
            }
        }
        else if constexpr (std::is_same_v<T, ForStmt>) {
            m_currentLine = s.sourceLine;
            if (m_lineCallback) m_lineCallback(s.sourceLine);

            double start = evaluateExpression(s.start);
            double end = evaluateExpression(s.end);
            double step = s.step ? evaluateExpression(s.step) : 1.0;

            setVariable(s.counter, start);
            size_t iteration = 0;

            auto condition = [&]() {
                return step > 0 ? getVariable(s.counter) <= end
                                : getVariable(s.counter) >= end;
            };

            while (condition()) {
                checkPauseOrStop();
                if (++iteration > m_maxLoopIterations) {
                    throw std::runtime_error("FOR iteration limit exceeded at line " +
                        std::to_string(s.sourceLine));
                }
                try {
                    executeStatements(s.body);
                } catch (const ExitLoopException&) {
                    break;
                } catch (const ContinueLoopException&) {
                    // Fall through to increment
                }
                setVariable(s.counter, getVariable(s.counter) + step);
            }
        }
        else if constexpr (std::is_same_v<T, RepeatStmt>) {
            m_currentLine = s.sourceLine;
            if (m_lineCallback) m_lineCallback(s.sourceLine);

            size_t iteration = 0;
            do {
                checkPauseOrStop();
                if (++iteration > m_maxLoopIterations) {
                    throw std::runtime_error("REPEAT iteration limit exceeded at line " +
                        std::to_string(s.sourceLine));
                }
                try {
                    executeStatements(s.body);
                } catch (const ExitLoopException&) {
                    break;
                } catch (const ContinueLoopException&) {
                    continue;
                }
            } while (!evaluateCondition(s.condition));
        }
        else if constexpr (std::is_same_v<T, SwitchStmt>) {
            m_currentLine = s.sourceLine;
            if (m_lineCallback) m_lineCallback(s.sourceLine);

            double selector = evaluateExpression(s.selector);
            bool matched = false;

            for (const auto& clause : s.cases) {
                for (const auto& val : clause.values) {
                    double caseVal = evaluateExpression(val);
                    if (std::abs(selector - caseVal) < 1e-9) {
                        matched = true;
                        break;
                    }
                }
                if (matched) {
                    executeStatements(clause.body);
                    break;
                }
            }

            if (!matched && !s.defaultBody.empty()) {
                executeStatements(s.defaultBody);
            }
        }
        else if constexpr (std::is_same_v<T, GotoStmt>) {
            m_currentLine = s.sourceLine;
            LOG_DEBUG("GOTO {} at line {}", s.label, s.sourceLine);
            throw GotoException(s.label);
        }
        else if constexpr (std::is_same_v<T, LabelStmt>) {
            // Labels are no-ops during sequential execution (resolved by GOTO)
        }
        else if constexpr (std::is_same_v<T, HaltStmt>) {
            m_currentLine = s.sourceLine;
            if (m_lineCallback) m_lineCallback(s.sourceLine);
            LOG_INFO("HALT at line {}", s.sourceLine);
            m_state = ExecutionState::PAUSED;
            checkPauseOrStop();
        }
        else if constexpr (std::is_same_v<T, ContinueStmt>) {
            m_currentLine = s.sourceLine;
            LOG_DEBUG("CONTINUE at line {}", s.sourceLine);
            throw ContinueLoopException();
        }
        else if constexpr (std::is_same_v<T, ExitStmt>) {
            m_currentLine = s.sourceLine;
            LOG_DEBUG("EXIT at line {}", s.sourceLine);
            throw ExitLoopException();
        }
        else if constexpr (std::is_same_v<T, ProgramStmt>) {
            // Should not happen at statement level
        }
    }, *stmt);
}

// ============================================================================
// Expression Evaluation
// ============================================================================

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
            // Handle TOOL_DATA[N] and BASE_DATA[N] composite names from parser
            if (e.name.find("TOOL_DATA[") == 0 || e.name.find("BASE_DATA[") == 0) {
                auto bracketPos = e.name.find('[');
                auto endPos = e.name.find(']');
                if (bracketPos != std::string::npos && endPos != std::string::npos) {
                    std::string indexStr = e.name.substr(bracketPos + 1, endPos - bracketPos - 1);
                    return std::stod(indexStr);
                }
                return 0.0;
            }
            return getVariable(e.name);
        }
        else if constexpr (std::is_same_v<T, SystemVarExpr>) {
            std::string varName = e.name;
            std::transform(varName.begin(), varName.end(), varName.begin(), ::toupper);

            if (varName == "IN") {
                int index = e.index ? static_cast<int>(evaluateExpression(e.index)) : 0;
                return getInput(index) ? 1.0 : 0.0;
            }
            if (varName == "OUT") {
                int index = e.index ? static_cast<int>(evaluateExpression(e.index)) : 0;
                return getOutput(index) ? 1.0 : 0.0;
            }
            if (varName == "VEL_AXIS" || varName == "ACC_AXIS") {
                int index = e.index ? static_cast<int>(evaluateExpression(e.index)) : 1;
                if (index >= 1 && index <= 6) {
                    if (varName == "VEL_AXIS") return m_sysVars.velAxis[index - 1];
                    else return m_sysVars.accAxis[index - 1];
                }
                return 0.0;
            }
            if (varName == "POS_ACT") {
                int index = e.index ? static_cast<int>(evaluateExpression(e.index)) : 0;
                if (index > 0 && index <= static_cast<int>(m_currentPosition.size())) {
                    return m_currentPosition[index - 1];  // 1-indexed
                }
                return 0.0;
            }

            // All other system variables: $VEL.CP, $ACC.CP, $APO.CDIS, $OV_PRO, $TOOL, $BASE, etc.
            return getSystemVariable(varName, e.subField);
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
            if (e.op == "EXOR") return ((left != 0) != (right != 0)) ? 1.0 : 0.0;
            if (e.op == "B_AND") return static_cast<double>(static_cast<int>(left) & static_cast<int>(right));
            if (e.op == "B_OR") return static_cast<double>(static_cast<int>(left) | static_cast<int>(right));
            if (e.op == "B_EXOR") return static_cast<double>(static_cast<int>(left) ^ static_cast<int>(right));

            return 0.0;
        }
        else if constexpr (std::is_same_v<T, UnaryExpr>) {
            double operand = evaluateExpression(e.operand);

            if (e.op == "-") return -operand;
            if (e.op == "NOT") return operand == 0 ? 1.0 : 0.0;
            if (e.op == "B_NOT") return static_cast<double>(~static_cast<int>(operand));

            return operand;
        }
        else if constexpr (std::is_same_v<T, PointExpr>) {
            // Points don't evaluate to a single number in numeric context
            // Motion target resolution is done in the motion callback
            return 0.0;
        }
        else if constexpr (std::is_same_v<T, AggregateExpr>) {
            // Aggregates represent structured data, not a single number
            // Their field values are evaluated when needed by motion/assignment handlers
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
        else if constexpr (std::is_same_v<T, ForStmt>) return s.sourceLine;
        else if constexpr (std::is_same_v<T, RepeatStmt>) return s.sourceLine;
        else if constexpr (std::is_same_v<T, SwitchStmt>) return s.sourceLine;
        else if constexpr (std::is_same_v<T, GotoStmt>) return s.sourceLine;
        else if constexpr (std::is_same_v<T, LabelStmt>) return s.sourceLine;
        else if constexpr (std::is_same_v<T, HaltStmt>) return s.sourceLine;
        else if constexpr (std::is_same_v<T, ContinueStmt>) return s.sourceLine;
        else if constexpr (std::is_same_v<T, ExitStmt>) return s.sourceLine;
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

    if (auto* ms = std::get_if<MotionStmt>(stmt.get())) {
        snap.wasMotion = true;
        snap.motionType = ms->type;
    }

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

    m_pc = targetPC;
    m_currentLine = sourceLine;
    m_undoStack.clear();
    m_state = ExecutionState::PAUSED;

    if (m_lineCallback) m_lineCallback(sourceLine);
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

    m_variables = std::move(snap.variables);

    for (int i = 0; i < 128; ++i) {
        if (m_outputs[i] != snap.outputs[i]) {
            m_outputs[i] = snap.outputs[i];
            if (m_outputCallback) m_outputCallback(i, snap.outputs[i]);
        }
    }

    if (snap.wasMotion && !snap.prevPosition.empty()) {
        MotionStmt reverseMotion;
        reverseMotion.type = snap.motionType;
        reverseMotion.sourceLine = snap.sourceLine;
        reverseMotion.target = makeExpr(PointExpr{"_BWD_TARGET", snap.prevPosition, false});

        if (m_motionCallback) m_motionCallback(reverseMotion);
        m_currentPosition = snap.prevPosition;
    }

    m_pc = snap.pc;
    m_currentLine = snap.sourceLine;
    m_state = ExecutionState::PAUSED;

    if (m_lineCallback) m_lineCallback(snap.sourceLine);
    LOG_INFO("BWD: reversed to line {}, pc={}", snap.sourceLine, snap.pc);
}

// ============================================================================
// Point Value Parser (extracts numeric values from aggregate/bracket formats)
// ============================================================================

std::vector<double> Executor::parseRobtargetValues(const std::string& raw) {
    std::vector<double> result;
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

    if (allNumbers.size() >= 3) {
        result = allNumbers;
    }

    return result;
}

} // namespace interpreter
} // namespace robot_controller
