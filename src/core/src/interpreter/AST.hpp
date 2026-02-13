#pragma once

/**
 * @file AST.hpp
 * @brief Abstract Syntax Tree definitions for KRL (KUKA Robot Language)
 *
 * Strict KUKA KRL syntax based on KSS 8.x specifications.
 */

#include <memory>
#include <vector>
#include <string>
#include <variant>
#include <optional>

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
struct AggregateExpr;

// Expression variant type
using Expression = std::variant<
    LiteralExpr,
    VariableExpr,
    SystemVarExpr,
    BinaryExpr,
    UnaryExpr,
    PointExpr,
    AggregateExpr
>;

using ExprPtr = std::shared_ptr<Expression>;

// ============================================================================
// STATEMENT TYPES (forward declarations)
// ============================================================================

struct ProgramStmt;
struct DeclareStmt;
struct ConstDeclStmt;
struct FunctionCallStmt;
struct AssignStmt;
struct SystemAssignStmt;
struct MotionStmt;
struct WaitStmt;
struct IfStmt;
struct LoopStmt;
struct WhileStmt;
struct ForStmt;
struct RepeatStmt;
struct SwitchStmt;
struct GotoStmt;
struct LabelStmt;
struct HaltStmt;
struct ContinueStmt;
struct ExitStmt;

// Statement variant type
using Statement = std::variant<
    ProgramStmt,
    DeclareStmt,
    ConstDeclStmt,
    FunctionCallStmt,
    AssignStmt,
    SystemAssignStmt,
    MotionStmt,
    WaitStmt,
    IfStmt,
    LoopStmt,
    WhileStmt,
    ForStmt,
    RepeatStmt,
    SwitchStmt,
    GotoStmt,
    LabelStmt,
    HaltStmt,
    ContinueStmt,
    ExitStmt
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
    std::string name;      // VEL, ACC, APO, TOOL, BASE, IN, OUT, OV_PRO, etc.
    std::string subField;  // CP, CDIS, CPTP, CORI, CVEL, etc. (for $VEL.CP, $APO.CDIS)
    ExprPtr index;         // Optional index for arrays ($IN[1], $VEL_AXIS[1])
};

struct BinaryExpr {
    ExprPtr left;
    std::string op;        // +, -, *, /, ==, <>, <, >, AND, OR, EXOR, B_AND, B_OR, B_EXOR, :
    ExprPtr right;
};

struct UnaryExpr {
    std::string op;        // -, NOT, B_NOT
    ExprPtr operand;
};

struct PointExpr {
    std::string name;      // P1, HOME, etc.
    std::vector<double> values;
    bool isJoint = false;  // Joint angles vs Cartesian
};

// KRL aggregate initializer: {X 10, Y 20, Z 30, A 0, B 90, C 0}
struct AggregateExpr {
    std::string typeName;  // Optional: "E6POS", "POS", etc. (from {E6POS: ...})
    std::vector<std::pair<std::string, ExprPtr>> fields;  // {name, value} pairs
};

// ============================================================================
// STATEMENT DEFINITIONS
// ============================================================================

struct ProgramStmt {
    std::string name;
    std::vector<StmtPtr> constDecls;  // Top-level CONST/DECL before DEF
    std::vector<StmtPtr> body;
};

struct DeclareStmt {
    std::string type;      // REAL, INT, BOOL, CHAR, FRAME, POS, E6POS, E6AXIS, AXIS
    std::string name;
    int arraySize = 0;     // >0 for array declarations: DECL INT arr[10]
    ExprPtr initializer;   // Optional
};

struct ConstDeclStmt {
    std::string type;      // "E6POS", "INT", "REAL", etc.
    std::string name;
    std::string rawValue;  // "{X 10, Y 20, Z 30, ...}" or "42"
    int sourceLine = 0;
};

struct FunctionCallStmt {
    std::string name;
    std::vector<std::pair<std::string, ExprPtr>> args;
    int sourceLine = 0;
};

struct AssignStmt {
    std::string name;
    ExprPtr value;
};

struct SystemAssignStmt {
    std::string name;      // OUT, VEL, ACC, APO, TOOL, BASE, OV_PRO, etc.
    std::string subField;  // CP, CDIS, CPTP, etc. (for $VEL.CP = 0.1)
    ExprPtr index;         // Optional index for arrays ($OUT[1], $VEL_AXIS[1])
    ExprPtr value;
};

struct MotionStmt {
    std::string type;      // PTP, LIN, CIRC, PTP_REL, LIN_REL, CIRC_REL
    ExprPtr target;        // Point expression or aggregate
    ExprPtr auxPoint;      // For CIRC/CIRC_REL (auxiliary/via point)
    ExprPtr circAngle;     // For CIRC/CIRC_REL: optional CA angle (degrees)

    // KRL approximation (from instruction keyword, velocity/tool via system variables)
    bool continuous = false;
    std::string approxType;  // "C_PTP", "C_DIS", "C_VEL", "C_ORI" or empty

    int sourceLine = 0;
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
    std::vector<StmtPtr> elseBranch;
    int sourceLine = 0;
};

struct LoopStmt {
    ExprPtr count;         // nullptr for infinite (KRL LOOP is always infinite)
    std::vector<StmtPtr> body;
    int sourceLine = 0;
};

struct WhileStmt {
    ExprPtr condition;
    std::vector<StmtPtr> body;
    int sourceLine = 0;
};

// FOR counter = start TO end STEP increment ... ENDFOR
struct ForStmt {
    std::string counter;
    ExprPtr start;
    ExprPtr end;
    ExprPtr step;          // Optional, default 1
    std::vector<StmtPtr> body;
    int sourceLine = 0;
};

// REPEAT ... UNTIL condition
struct RepeatStmt {
    std::vector<StmtPtr> body;
    ExprPtr condition;     // Loop exits when condition becomes TRUE
    int sourceLine = 0;
};

// SWITCH expr CASE val1 ... CASE val2 ... DEFAULT ... ENDSWITCH
struct SwitchStmt {
    ExprPtr selector;
    struct CaseClause {
        std::vector<ExprPtr> values;   // Multiple values per CASE (CASE 1, 2, 3)
        std::vector<StmtPtr> body;
    };
    std::vector<CaseClause> cases;
    std::vector<StmtPtr> defaultBody;  // Optional DEFAULT clause
    int sourceLine = 0;
};

// GOTO label
struct GotoStmt {
    std::string label;
    int sourceLine = 0;
};

// label:
struct LabelStmt {
    std::string name;
    int sourceLine = 0;
};

// HALT
struct HaltStmt {
    int sourceLine = 0;
};

// CONTINUE (prevents advance run stop)
struct ContinueStmt {
    int sourceLine = 0;
};

// EXIT (exits loop prematurely)
struct ExitStmt {
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
