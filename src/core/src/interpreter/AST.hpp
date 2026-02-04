#pragma once

/**
 * @file AST.hpp
 * @brief Abstract Syntax Tree definitions for RPL
 *
 * Part of Phase 8: Virtual Simulation (IMPL_P8_02)
 */

#include <memory>
#include <vector>
#include <string>
#include <variant>

namespace robot_controller {
namespace interpreter {

// ============================================================================
// EXPRESSION TYPES (forward declarations for recursive types)
// ============================================================================

struct LiteralExpr;
struct VariableExpr;
struct SystemVarExpr;
struct BinaryExpr;
struct UnaryExpr;
struct PointExpr;

// Expression variant type
using Expression = std::variant<
    LiteralExpr,
    VariableExpr,
    SystemVarExpr,
    BinaryExpr,
    UnaryExpr,
    PointExpr
>;

using ExprPtr = std::shared_ptr<Expression>;

// ============================================================================
// STATEMENT TYPES (forward declarations)
// ============================================================================

struct ProgramStmt;
struct DeclareStmt;
struct AssignStmt;
struct SystemAssignStmt;
struct MotionStmt;
struct WaitStmt;
struct IfStmt;
struct LoopStmt;
struct WhileStmt;

// Statement variant type
using Statement = std::variant<
    ProgramStmt,
    DeclareStmt,
    AssignStmt,
    SystemAssignStmt,
    MotionStmt,
    WaitStmt,
    IfStmt,
    LoopStmt,
    WhileStmt
>;

using StmtPtr = std::shared_ptr<Statement>;

// ============================================================================
// EXPRESSION DEFINITIONS
// ============================================================================

struct LiteralExpr {
    std::variant<double, int, bool, std::string> value;
};

struct VariableExpr {
    std::string name;
};

struct SystemVarExpr {
    std::string name;      // IN, OUT, VEL, etc.
    ExprPtr index;         // Optional index for arrays
};

struct BinaryExpr {
    ExprPtr left;
    std::string op;        // +, -, *, /, ==, <, >, etc.
    ExprPtr right;
};

struct UnaryExpr {
    std::string op;        // -, NOT
    ExprPtr operand;
};

struct PointExpr {
    std::string name;      // P1, HOME, etc.
    // Or inline definition
    std::vector<double> values;
    bool isJoint = false;  // Joint angles vs Cartesian
};

// ============================================================================
// STATEMENT DEFINITIONS
// ============================================================================

struct ProgramStmt {
    std::string name;
    std::vector<StmtPtr> body;
};

struct DeclareStmt {
    std::string type;      // REAL, INT, BOOL
    std::string name;
    ExprPtr initializer;   // Optional
};

struct AssignStmt {
    std::string name;
    ExprPtr value;
};

struct SystemAssignStmt {
    std::string name;      // OUT
    ExprPtr index;
    ExprPtr value;
};

struct MotionStmt {
    std::string type;      // PTP, LIN, CIRC
    ExprPtr target;        // Point expression
    ExprPtr auxPoint;      // For CIRC

    // Optional parameters
    double velocity = 100;
    std::string velocityUnit = "percent";  // percent, mm/s
    double acceleration = 100;
    bool continuous = false;

    int sourceLine = 0;    // For debugging
};

struct WaitStmt {
    enum class WaitType { TIME, CONDITION };
    WaitType type = WaitType::TIME;
    ExprPtr value;         // Seconds or condition

    int sourceLine = 0;
};

struct IfStmt {
    ExprPtr condition;
    std::vector<StmtPtr> thenBranch;
    std::vector<StmtPtr> elseBranch;  // Optional

    int sourceLine = 0;
};

struct LoopStmt {
    ExprPtr count;         // Number of iterations, or nullptr for infinite
    std::vector<StmtPtr> body;

    int sourceLine = 0;
};

struct WhileStmt {
    ExprPtr condition;
    std::vector<StmtPtr> body;

    int sourceLine = 0;
};

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

template<typename T>
ExprPtr makeExpr(T&& expr) {
    return std::make_shared<Expression>(std::forward<T>(expr));
}

template<typename T>
StmtPtr makeStmt(T&& stmt) {
    return std::make_shared<Statement>(std::forward<T>(stmt));
}

} // namespace interpreter
} // namespace robot_controller
