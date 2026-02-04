# IMPL_P8_02: Virtual Simulation - Program Interpreter

| Metadata | Value |
|----------|-------|
| Phase | Phase 8: Virtual Simulation |
| Plan ID | IMPL_P8_02 |
| Title | Program Interpreter (RPL Language) |
| Priority | P0 (Core for simulation) |
| Status | PLANNED |
| Created | 2026-02-04 |
| Depends On | IMPL_P8_01 |

---

## 1. Overview

### 1.1 Mục tiêu

Xây dựng interpreter cho Robot Programming Language (RPL) - ngôn ngữ lập trình robot đơn giản lấy cảm hứng từ KUKA KRL.

### 1.2 Scope

- Lexer (tokenizer)
- Parser (AST generation)
- Executor (step/run/pause/stop)
- Point database integration
- I/O simulation

### 1.3 Language Features (Phase 1)

| Feature | Syntax | Example |
|---------|--------|---------|
| Motion | PTP, LIN, CIRC | `PTP P1 VEL=100%` |
| Variables | DECL, assignment | `DECL REAL x = 10` |
| Control | IF/LOOP/WHILE | `IF $IN[1] THEN` |
| Wait | WAIT SEC, WAIT FOR | `WAIT SEC 1.5` |
| I/O | $IN, $OUT | `$OUT[1] = TRUE` |
| Comments | ; | `; this is comment` |

---

## 2. Prerequisites

- [ ] IMPL_P8_01 hoàn thành
- [ ] Trajectory generator đã có (Phase 2)
- [ ] Kinematics service đã có (Phase 2)

---

## 3. Implementation Steps

### Step 1: Define Token Types

**Files to create:**
- `src/core/src/interpreter/Token.hpp`

```cpp
#pragma once

#include <string>
#include <variant>

namespace robot_controller {
namespace interpreter {

enum class TokenType {
    // Literals
    NUMBER,         // 123, 45.67
    STRING,         // "hello"
    IDENTIFIER,     // variable names, point names

    // Keywords
    DEF, END,
    DECL, REAL, INT, BOOL,
    IF, THEN, ELSE, ENDIF,
    LOOP, ENDLOOP,
    WHILE, ENDWHILE,
    WAIT, SEC, FOR,
    TRUE, FALSE,

    // Motion
    PTP, LIN, CIRC,
    VEL, ACC, CONT,
    HOME,

    // Operators
    PLUS, MINUS, STAR, SLASH,
    EQUAL, NOT_EQUAL,
    LESS, GREATER, LESS_EQ, GREATER_EQ,
    ASSIGN,         // =
    AND, OR, NOT,

    // Punctuation
    LPAREN, RPAREN,
    LBRACE, RBRACE,
    LBRACKET, RBRACKET,
    COMMA, COLON,
    PERCENT,
    DOLLAR,         // $

    // Special
    NEWLINE,
    COMMENT,
    END_OF_FILE,
    ERROR
};

struct Token {
    TokenType type;
    std::string lexeme;
    std::variant<std::monostate, double, int, bool, std::string> literal;
    int line;
    int column;
};

std::string tokenTypeToString(TokenType type);

} // namespace interpreter
} // namespace robot_controller
```

**Validation:**
- Header compiles

---

### Step 2: Implement Lexer

**Files to create:**
- `src/core/src/interpreter/Lexer.hpp`
- `src/core/src/interpreter/Lexer.cpp`

```cpp
#pragma once

#include "Token.hpp"
#include <vector>
#include <string>
#include <unordered_map>

namespace robot_controller {
namespace interpreter {

class Lexer {
public:
    explicit Lexer(const std::string& source);

    std::vector<Token> tokenize();

    const std::vector<std::string>& getErrors() const { return m_errors; }
    bool hasErrors() const { return !m_errors.empty(); }

private:
    std::string m_source;
    std::vector<Token> m_tokens;
    std::vector<std::string> m_errors;

    size_t m_start = 0;
    size_t m_current = 0;
    int m_line = 1;
    int m_column = 1;

    static const std::unordered_map<std::string, TokenType> s_keywords;

    bool isAtEnd() const;
    char advance();
    char peek() const;
    char peekNext() const;
    bool match(char expected);

    void scanToken();
    void addToken(TokenType type);
    void addToken(TokenType type, std::variant<std::monostate, double, int, bool, std::string> literal);

    void scanNumber();
    void scanString();
    void scanIdentifier();
    void scanComment();

    void error(const std::string& message);
};

} // namespace interpreter
} // namespace robot_controller
```

**Key Implementation:**

```cpp
void Lexer::scanToken() {
    char c = advance();

    switch (c) {
        // Single-character tokens
        case '(': addToken(TokenType::LPAREN); break;
        case ')': addToken(TokenType::RPAREN); break;
        case '{': addToken(TokenType::LBRACE); break;
        case '}': addToken(TokenType::RBRACE); break;
        case '[': addToken(TokenType::LBRACKET); break;
        case ']': addToken(TokenType::RBRACKET); break;
        case ',': addToken(TokenType::COMMA); break;
        case ':': addToken(TokenType::COLON); break;
        case '+': addToken(TokenType::PLUS); break;
        case '-': addToken(TokenType::MINUS); break;
        case '*': addToken(TokenType::STAR); break;
        case '/': addToken(TokenType::SLASH); break;
        case '%': addToken(TokenType::PERCENT); break;
        case '$': addToken(TokenType::DOLLAR); break;

        case '=':
            addToken(match('=') ? TokenType::EQUAL : TokenType::ASSIGN);
            break;

        case '<':
            if (match('=')) addToken(TokenType::LESS_EQ);
            else if (match('>')) addToken(TokenType::NOT_EQUAL);
            else addToken(TokenType::LESS);
            break;

        case '>':
            addToken(match('=') ? TokenType::GREATER_EQ : TokenType::GREATER);
            break;

        case ';':
            scanComment();
            break;

        case '"':
            scanString();
            break;

        case '\n':
            addToken(TokenType::NEWLINE);
            m_line++;
            m_column = 1;
            break;

        case ' ':
        case '\r':
        case '\t':
            // Ignore whitespace
            break;

        default:
            if (isdigit(c)) {
                scanNumber();
            } else if (isalpha(c) || c == '_') {
                scanIdentifier();
            } else {
                error("Unexpected character: " + std::string(1, c));
            }
            break;
    }
}
```

**Validation:**
```cpp
// Test lexer
Lexer lexer("PTP P1 VEL=100%");
auto tokens = lexer.tokenize();
// Expected: PTP, IDENTIFIER(P1), IDENTIFIER(VEL), ASSIGN, NUMBER(100), PERCENT
```

---

### Step 3: Define AST Nodes

**Files to create:**
- `src/core/src/interpreter/AST.hpp`

```cpp
#pragma once

#include <memory>
#include <vector>
#include <string>
#include <variant>

namespace robot_controller {
namespace interpreter {

// Forward declarations
struct Expression;
struct Statement;

using ExprPtr = std::shared_ptr<Expression>;
using StmtPtr = std::shared_ptr<Statement>;

// ============================================================================
// EXPRESSIONS
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
    bool isJoint;          // Joint angles vs Cartesian
};

using Expression = std::variant<
    LiteralExpr,
    VariableExpr,
    SystemVarExpr,
    BinaryExpr,
    UnaryExpr,
    PointExpr
>;

// ============================================================================
// STATEMENTS
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
};

struct WaitStmt {
    enum class WaitType { TIME, CONDITION };
    WaitType type;
    ExprPtr value;         // Seconds or condition
};

struct IfStmt {
    ExprPtr condition;
    std::vector<StmtPtr> thenBranch;
    std::vector<StmtPtr> elseBranch;  // Optional
};

struct LoopStmt {
    ExprPtr count;         // Number of iterations, or nullptr for infinite
    std::vector<StmtPtr> body;
};

struct WhileStmt {
    ExprPtr condition;
    std::vector<StmtPtr> body;
};

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

} // namespace interpreter
} // namespace robot_controller
```

**Validation:**
- Header compiles

---

### Step 4: Implement Parser

**Files to create:**
- `src/core/src/interpreter/Parser.hpp`
- `src/core/src/interpreter/Parser.cpp`

```cpp
#pragma once

#include "Token.hpp"
#include "AST.hpp"
#include <vector>
#include <optional>

namespace robot_controller {
namespace interpreter {

class Parser {
public:
    explicit Parser(const std::vector<Token>& tokens);

    std::optional<ProgramStmt> parse();

    const std::vector<std::string>& getErrors() const { return m_errors; }
    bool hasErrors() const { return !m_errors.empty(); }

private:
    std::vector<Token> m_tokens;
    std::vector<std::string> m_errors;
    size_t m_current = 0;

    // Helpers
    bool isAtEnd() const;
    Token peek() const;
    Token previous() const;
    Token advance();
    bool check(TokenType type) const;
    bool match(std::initializer_list<TokenType> types);
    Token consume(TokenType type, const std::string& message);
    void error(const std::string& message);
    void synchronize();

    // Grammar rules
    ProgramStmt parseProgram();
    StmtPtr parseStatement();
    StmtPtr parseDeclaration();
    StmtPtr parseAssignment();
    StmtPtr parseMotion();
    StmtPtr parseWait();
    StmtPtr parseIf();
    StmtPtr parseLoop();
    StmtPtr parseWhile();

    ExprPtr parseExpression();
    ExprPtr parseOr();
    ExprPtr parseAnd();
    ExprPtr parseEquality();
    ExprPtr parseComparison();
    ExprPtr parseTerm();
    ExprPtr parseFactor();
    ExprPtr parseUnary();
    ExprPtr parsePrimary();
    ExprPtr parsePoint();
    ExprPtr parseSystemVar();
};

} // namespace interpreter
} // namespace robot_controller
```

**Validation:**
```cpp
// Test parser
std::string source = R"(
DEF TestProgram()
    PTP HOME
    PTP P1 VEL=80%
    LIN P2 VEL=10mm/s
END
)";

Lexer lexer(source);
Parser parser(lexer.tokenize());
auto program = parser.parse();
// Should produce valid AST
```

---

### Step 5: Implement Executor

**Files to create:**
- `src/core/src/interpreter/Executor.hpp`
- `src/core/src/interpreter/Executor.cpp`

```cpp
#pragma once

#include "AST.hpp"
#include <functional>
#include <unordered_map>
#include <atomic>

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

class Executor {
public:
    using MotionCallback = std::function<void(const MotionStmt&)>;
    using LineCallback = std::function<void(int line)>;
    using WaitCallback = std::function<void(double seconds)>;

    Executor();

    // Load program
    void loadProgram(const ProgramStmt& program);

    // Execution control
    void run();
    void step();      // Execute one statement
    void pause();
    void resume();
    void stop();
    void reset();

    // State
    ExecutionState getState() const { return m_state; }
    int getCurrentLine() const { return m_currentLine; }

    // Callbacks
    void setMotionCallback(MotionCallback cb) { m_motionCallback = cb; }
    void setLineCallback(LineCallback cb) { m_lineCallback = cb; }
    void setWaitCallback(WaitCallback cb) { m_waitCallback = cb; }

    // Point database
    void setPoint(const std::string& name, const std::vector<double>& values);
    std::vector<double> getPoint(const std::string& name) const;

    // Variables
    void setVariable(const std::string& name, double value);
    double getVariable(const std::string& name) const;

    // I/O
    void setInput(int index, bool value);
    bool getInput(int index) const;
    void setOutput(int index, bool value);
    bool getOutput(int index) const;

private:
    ProgramStmt m_program;
    std::atomic<ExecutionState> m_state{ExecutionState::IDLE};
    size_t m_pc = 0;  // Program counter
    int m_currentLine = 0;

    // Data
    std::unordered_map<std::string, std::vector<double>> m_points;
    std::unordered_map<std::string, double> m_variables;
    std::array<bool, 128> m_inputs{};
    std::array<bool, 128> m_outputs{};

    // Callbacks
    MotionCallback m_motionCallback;
    LineCallback m_lineCallback;
    WaitCallback m_waitCallback;

    // Execution
    void executeStatement(const StmtPtr& stmt);
    double evaluateExpression(const ExprPtr& expr);
    bool evaluateCondition(const ExprPtr& expr);
};

} // namespace interpreter
} // namespace robot_controller
```

**Validation:**
- Executor can run simple programs
- Step mode works correctly

---

### Step 6: Integrate with Virtual Controller

**Files to modify:**
- `src/core/src/controller/RobotController.hpp`
- `src/core/src/controller/RobotController.cpp`

**Add interpreter integration:**

```cpp
// RobotController.hpp
#include "../interpreter/Executor.hpp"

class RobotController {
    // ...
    std::unique_ptr<interpreter::Executor> m_programExecutor;

    // Program execution
    bool loadProgram(const std::string& source);
    void runProgram();
    void stepProgram();
    void pauseProgram();
    void stopProgram();

    interpreter::ExecutionState getProgramState() const;
};
```

**Add IPC handlers:**

```cpp
// New message types
LOAD_PROGRAM,
RUN_PROGRAM,
STEP_PROGRAM,
PAUSE_PROGRAM,
STOP_PROGRAM,
GET_PROGRAM_STATE,
SET_POINT,
GET_POINTS
```

**Validation:**
- Load program via IPC
- Run/Step/Pause/Stop work
- Motion commands sent to trajectory generator

---

## 4. Completion Checklist

- [ ] Step 1: Token types defined
- [ ] Step 2: Lexer tokenizes RPL code
- [ ] Step 3: AST nodes defined
- [ ] Step 4: Parser creates AST
- [ ] Step 5: Executor runs programs
- [ ] Step 6: Integration with controller
- [ ] Unit tests for Lexer
- [ ] Unit tests for Parser
- [ ] Integration test: full program execution

---

## 5. Test Programs

### Test 1: Basic Motion

```
DEF TestMotion()
    PTP HOME
    PTP P1 VEL=50%
    LIN P2 VEL=100mm/s
    PTP HOME
END
```

### Test 2: Control Flow

```
DEF TestLoop()
    DECL INT i = 0
    LOOP 3
        PTP P1
        PTP P2
        WAIT SEC 0.5
    ENDLOOP
END
```

### Test 3: I/O

```
DEF TestIO()
    IF $IN[1] == TRUE THEN
        $OUT[1] = TRUE
        WAIT SEC 2
        $OUT[1] = FALSE
    ENDIF
END
```

---

## 6. Next Steps

- IMPL_P8_03: Program Editor UI (WPF with syntax highlighting)
- IMPL_P8_04: Export System
