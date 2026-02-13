/**
 * @file Parser.cpp
 * @brief Parser implementation for KRL (KUKA Robot Language)
 *
 * Strict KUKA KRL syntax based on KSS 8.x specifications.
 */

#include "Parser.hpp"
#include <stdexcept>
#include <algorithm>

namespace robot_controller {
namespace interpreter {

Parser::Parser(const std::vector<Token>& tokens)
    : m_tokens(tokens) {}

std::optional<ProgramStmt> Parser::parse() {
    try {
        skipNewlines();
        return parseProgram();
    } catch (const std::exception& e) {
        error(e.what());
        return std::nullopt;
    }
}

bool Parser::isAtEnd() const {
    return peek().type == TokenType::END_OF_FILE;
}

Token Parser::peek() const {
    return m_tokens[m_current];
}

Token Parser::previous() const {
    return m_tokens[m_current - 1];
}

Token Parser::advance() {
    if (!isAtEnd()) m_current++;
    return previous();
}

bool Parser::check(TokenType type) const {
    if (isAtEnd()) return false;
    return peek().type == type;
}

bool Parser::match(std::initializer_list<TokenType> types) {
    for (TokenType type : types) {
        if (check(type)) {
            advance();
            return true;
        }
    }
    return false;
}

Token Parser::consume(TokenType type, const std::string& message) {
    if (check(type)) return advance();
    error(message + " at line " + std::to_string(peek().line));
    throw std::runtime_error(message);
}

void Parser::error(const std::string& message) {
    m_errors.push_back("Line " + std::to_string(peek().line) + ": " + message);
}

void Parser::synchronize() {
    advance();
    while (!isAtEnd()) {
        if (previous().type == TokenType::NEWLINE) return;
        switch (peek().type) {
            case TokenType::DEF:
            case TokenType::IF:
            case TokenType::LOOP:
            case TokenType::WHILE:
            case TokenType::FOR:
            case TokenType::REPEAT:
            case TokenType::SWITCH:
            case TokenType::PTP:
            case TokenType::LIN:
            case TokenType::CIRC:
            case TokenType::PTP_REL:
            case TokenType::LIN_REL:
            case TokenType::CIRC_REL:
            case TokenType::WAIT:
            case TokenType::DECL:
            case TokenType::CONST:
            case TokenType::HALT:
            case TokenType::GOTO:
            case TokenType::CONTINUE:
            case TokenType::EXIT:
                return;
            default:
                advance();
        }
    }
}

void Parser::skipNewlines() {
    while (match({TokenType::NEWLINE})) {}
}

bool Parser::isKrlType() const {
    return check(TokenType::INT) || check(TokenType::REAL) || check(TokenType::BOOL) ||
           check(TokenType::CHAR) || check(TokenType::FRAME) || check(TokenType::POS) ||
           check(TokenType::E6POS) || check(TokenType::E6AXIS) || check(TokenType::AXIS);
}

// ============================================================================
// Program Structure
// ============================================================================

ProgramStmt Parser::parseProgram() {
    ProgramStmt program;

    skipNewlines();

    // Top-level declarations before DEF (CONST, DECL)
    while ((check(TokenType::CONST) || check(TokenType::DECL)) && !isAtEnd()) {
        try {
            StmtPtr decl;
            if (check(TokenType::CONST))
                decl = parseConstDeclaration();
            else
                decl = parseDeclaration();
            if (decl) {
                program.constDecls.push_back(decl);
            }
        } catch (const std::exception&) {
            synchronize();
        }
        skipNewlines();
    }

    // DEF name() ... END
    consume(TokenType::DEF, "Expected 'DEF'");
    Token name = consume(TokenType::IDENTIFIER, "Expected program name");
    program.name = name.lexeme;

    consume(TokenType::LPAREN, "Expected '('");
    consume(TokenType::RPAREN, "Expected ')'");
    skipNewlines();

    // Body
    while (!check(TokenType::END) && !isAtEnd()) {
        try {
            auto stmt = parseStatement();
            if (stmt) {
                program.body.push_back(stmt);
            }
        } catch (const std::exception&) {
            synchronize();
        }
        skipNewlines();
    }

    consume(TokenType::END, "Expected 'END'");
    return program;
}

// ============================================================================
// Statement Dispatcher
// ============================================================================

StmtPtr Parser::parseStatement() {
    skipNewlines();

    if (check(TokenType::DECL)) return parseDeclaration();
    if (check(TokenType::CONST)) return parseConstDeclaration();

    // KRL motion: PTP, LIN, CIRC, PTP_REL, LIN_REL, CIRC_REL
    if (check(TokenType::PTP) || check(TokenType::LIN) || check(TokenType::CIRC) ||
        check(TokenType::PTP_REL) || check(TokenType::LIN_REL) || check(TokenType::CIRC_REL)) {
        TokenType type = peek().type;
        advance();
        return parseMotion(type);
    }

    if (check(TokenType::WAIT)) return parseWait();
    if (check(TokenType::IF)) return parseIf();
    if (check(TokenType::LOOP)) return parseLoop();
    if (check(TokenType::WHILE)) return parseWhile();
    if (check(TokenType::FOR)) return parseFor();
    if (check(TokenType::REPEAT)) return parseRepeat();
    if (check(TokenType::SWITCH)) return parseSwitch();
    if (check(TokenType::GOTO)) return parseGoto();
    if (check(TokenType::HALT)) return parseHalt();
    if (check(TokenType::CONTINUE)) return parseContinue();
    if (check(TokenType::EXIT)) return parseExit();

    if (check(TokenType::DOLLAR)) return parseSystemAssignment();

    if (check(TokenType::IDENTIFIER)) {
        Token name = advance();

        // Label: "name:"
        if (check(TokenType::COLON)) {
            advance(); // consume ':'
            LabelStmt stmt;
            stmt.name = name.lexeme;
            stmt.sourceLine = name.line;
            return makeStmt(stmt);
        }

        // Assignment: name = expr
        if (check(TokenType::ASSIGN)) {
            return parseAssignment(name.lexeme);
        }

        // Function call: name(...) or standalone name
        if (check(TokenType::LPAREN) || check(TokenType::NEWLINE) || isAtEnd()) {
            return parseFunctionCall(name.lexeme);
        }

        error("Unexpected identifier: " + name.lexeme);
        return nullptr;
    }

    if (check(TokenType::NEWLINE) || isAtEnd()) {
        return nullptr;
    }

    error("Unexpected token: " + peek().lexeme);
    return nullptr;
}

// ============================================================================
// Declarations
// ============================================================================

StmtPtr Parser::parseDeclaration() {
    int line = peek().line;
    consume(TokenType::DECL, "Expected 'DECL'");

    DeclareStmt stmt;

    // KRL types
    if (isKrlType()) {
        std::string upper = peek().lexeme;
        std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);
        stmt.type = upper;
        advance();
    } else if (check(TokenType::IDENTIFIER)) {
        stmt.type = advance().lexeme;
    } else {
        error("Expected type after DECL");
        return nullptr;
    }

    // Name
    Token name = consume(TokenType::IDENTIFIER, "Expected variable name");
    stmt.name = name.lexeme;

    // Optional array size: name[size]
    if (match({TokenType::LBRACKET})) {
        if (check(TokenType::NUMBER)) {
            Token sizeTok = advance();
            stmt.arraySize = static_cast<int>(std::get<double>(sizeTok.literal));
        }
        consume(TokenType::RBRACKET, "Expected ']'");
    }

    // Optional initializer: = value or = {aggregate}
    if (match({TokenType::ASSIGN})) {
        if (check(TokenType::LBRACE)) {
            stmt.initializer = parseAggregate();
        } else {
            stmt.initializer = parseExpression();
        }
    }

    return makeStmt(stmt);
}

StmtPtr Parser::parseConstDeclaration() {
    int line = peek().line;
    consume(TokenType::CONST, "Expected 'CONST'");

    ConstDeclStmt stmt;
    stmt.sourceLine = line;

    // Type
    if (isKrlType()) {
        std::string upper = peek().lexeme;
        std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);
        stmt.type = upper;
        advance();
    } else if (check(TokenType::IDENTIFIER)) {
        stmt.type = advance().lexeme;
    } else {
        error("Expected type after CONST");
        return nullptr;
    }

    // Name
    Token name = consume(TokenType::IDENTIFIER, "Expected variable name");
    stmt.name = name.lexeme;

    // KRL assignment: =
    if (!match({TokenType::ASSIGN})) {
        error("Expected '=' in CONST declaration");
        return nullptr;
    }

    // Capture raw value
    std::string rawValue;
    if (check(TokenType::LBRACE)) {
        int braceDepth = 0;
        while (!isAtEnd()) {
            Token tok = peek();
            if (tok.type == TokenType::LBRACE) {
                braceDepth++;
                rawValue += "{";
                advance();
            } else if (tok.type == TokenType::RBRACE) {
                braceDepth--;
                rawValue += "}";
                advance();
                if (braceDepth <= 0) break;
            } else if (tok.type == TokenType::NEWLINE) {
                break;
            } else {
                rawValue += tok.lexeme;
                if (tok.type != TokenType::COMMA) rawValue += " ";
                advance();
            }
        }
    } else if (check(TokenType::LBRACKET)) {
        int bracketDepth = 0;
        while (!isAtEnd()) {
            Token tok = peek();
            if (tok.type == TokenType::LBRACKET) {
                bracketDepth++;
                rawValue += "[";
                advance();
            } else if (tok.type == TokenType::RBRACKET) {
                bracketDepth--;
                rawValue += "]";
                advance();
                if (bracketDepth <= 0) break;
            } else if (tok.type == TokenType::COMMA) {
                rawValue += ",";
                advance();
            } else if (tok.type == TokenType::NUMBER) {
                rawValue += tok.lexeme;
                advance();
            } else if (tok.type == TokenType::MINUS) {
                rawValue += "-";
                advance();
            } else if (tok.type == TokenType::NEWLINE) {
                break;
            } else {
                rawValue += tok.lexeme;
                advance();
            }
        }
    } else {
        while (!check(TokenType::NEWLINE) && !isAtEnd()) {
            rawValue += peek().lexeme;
            advance();
        }
    }

    stmt.rawValue = rawValue;
    return makeStmt(stmt);
}

// ============================================================================
// Statements
// ============================================================================

StmtPtr Parser::parseFunctionCall(const std::string& name) {
    int line = previous().line;

    FunctionCallStmt stmt;
    stmt.name = name;
    stmt.sourceLine = line;

    if (match({TokenType::LPAREN})) {
        while (!check(TokenType::RPAREN) && !isAtEnd()) {
            ExprPtr argValue = parseExpression();
            stmt.args.push_back({"", argValue});
            if (!match({TokenType::COMMA})) break;
        }
        consume(TokenType::RPAREN, "Expected ')'");
    }

    return makeStmt(stmt);
}

StmtPtr Parser::parseAssignment(const std::string& name) {
    if (!match({TokenType::ASSIGN})) {
        error("Expected '='");
        return nullptr;
    }

    AssignStmt stmt;
    stmt.name = name;

    if (check(TokenType::LBRACE)) {
        stmt.value = parseAggregate();
    } else {
        stmt.value = parseExpression();
    }

    return makeStmt(stmt);
}

StmtPtr Parser::parseSystemAssignment() {
    int line = peek().line;
    consume(TokenType::DOLLAR, "Expected '$'");

    Token varName = consume(TokenType::IDENTIFIER, "Expected system variable name");

    SystemAssignStmt stmt;
    stmt.name = varName.lexeme;

    // Handle dotted notation: $VEL.CP, $APO.CDIS, $APO.CPTP
    if (match({TokenType::DOT})) {
        Token subField = consume(TokenType::IDENTIFIER, "Expected sub-field after '.'");
        stmt.subField = subField.lexeme;
    }

    // Optional index: $OUT[1], $VEL_AXIS[1]
    if (match({TokenType::LBRACKET})) {
        stmt.index = parseExpression();
        consume(TokenType::RBRACKET, "Expected ']'");
    }

    if (!match({TokenType::ASSIGN})) {
        error("Expected '='");
        return nullptr;
    }
    if (check(TokenType::LBRACE)) {
        stmt.value = parseAggregate();
    } else {
        stmt.value = parseExpression();
    }

    return makeStmt(stmt);
}

StmtPtr Parser::parseMotion(TokenType motionType) {
    int line = previous().line;

    MotionStmt stmt;
    stmt.sourceLine = line;

    switch (motionType) {
        case TokenType::PTP: stmt.type = "PTP"; break;
        case TokenType::LIN: stmt.type = "LIN"; break;
        case TokenType::CIRC: stmt.type = "CIRC"; break;
        case TokenType::PTP_REL: stmt.type = "PTP_REL"; break;
        case TokenType::LIN_REL: stmt.type = "LIN_REL"; break;
        case TokenType::CIRC_REL: stmt.type = "CIRC_REL"; break;
        default: break;
    }

    // Parse first argument (point/aggregate/HOME)
    auto parsePointArg = [&]() -> ExprPtr {
        if (check(TokenType::HOME)) {
            advance();
            PointExpr point;
            point.name = "HOME";
            return makeExpr(point);
        } else if (check(TokenType::LBRACE)) {
            return parseAggregate();
        } else if (check(TokenType::IDENTIFIER)) {
            Token name = advance();
            return parsePoint(name.lexeme);
        }
        error("Expected point name, HOME, or aggregate");
        return nullptr;
    };

    bool isCirc = (motionType == TokenType::CIRC || motionType == TokenType::CIRC_REL);

    if (isCirc) {
        // KRL CIRC syntax: CIRC AuxPoint, TargetPoint <, CA Angle> <C_DIS|C_ORI|C_VEL>
        stmt.auxPoint = parsePointArg();
        if (!stmt.auxPoint) return nullptr;

        if (!check(TokenType::COMMA)) {
            error("Expected ',' after auxiliary point in CIRC");
            return nullptr;
        }
        advance(); // consume comma

        stmt.target = parsePointArg();
        if (!stmt.target) return nullptr;

        // Optional: , CA angle
        if (check(TokenType::COMMA)) {
            advance(); // consume comma
            if (match({TokenType::CA})) {
                stmt.circAngle = parseExpression();
            }
        }
    } else {
        // PTP/LIN: single target point
        stmt.target = parsePointArg();
        if (!stmt.target) return nullptr;
    }

    // Optional approximation: C_PTP, C_DIS, C_VEL, C_ORI
    if (match({TokenType::C_PTP})) {
        stmt.continuous = true;
        stmt.approxType = "C_PTP";
    } else if (match({TokenType::C_DIS})) {
        stmt.continuous = true;
        stmt.approxType = "C_DIS";
    } else if (match({TokenType::C_VEL})) {
        stmt.continuous = true;
        stmt.approxType = "C_VEL";
    } else if (match({TokenType::C_ORI})) {
        stmt.continuous = true;
        stmt.approxType = "C_ORI";
    }

    return makeStmt(stmt);
}

StmtPtr Parser::parseWait() {
    int line = peek().line;
    consume(TokenType::WAIT, "Expected 'WAIT'");

    WaitStmt stmt;
    stmt.sourceLine = line;

    if (match({TokenType::SEC})) {
        stmt.type = WaitStmt::WaitType::TIME;
        stmt.value = parseExpression();
    } else if (match({TokenType::FOR})) {
        stmt.type = WaitStmt::WaitType::CONDITION;
        stmt.value = parseExpression();
    } else {
        error("Expected 'SEC' or 'FOR' after WAIT");
        return nullptr;
    }

    return makeStmt(stmt);
}

StmtPtr Parser::parseIf() {
    int line = peek().line;
    consume(TokenType::IF, "Expected 'IF'");

    IfStmt stmt;
    stmt.sourceLine = line;

    stmt.condition = parseExpression();
    consume(TokenType::THEN, "Expected 'THEN'");
    skipNewlines();

    while (!check(TokenType::ELSE) && !check(TokenType::ENDIF) && !isAtEnd()) {
        auto s = parseStatement();
        if (s) stmt.thenBranch.push_back(s);
        skipNewlines();
    }

    if (match({TokenType::ELSE})) {
        skipNewlines();
        while (!check(TokenType::ENDIF) && !isAtEnd()) {
            auto s = parseStatement();
            if (s) stmt.elseBranch.push_back(s);
            skipNewlines();
        }
    }

    consume(TokenType::ENDIF, "Expected 'ENDIF'");
    return makeStmt(stmt);
}

StmtPtr Parser::parseLoop() {
    int line = peek().line;
    consume(TokenType::LOOP, "Expected 'LOOP'");

    LoopStmt stmt;
    stmt.sourceLine = line;
    skipNewlines();

    while (!check(TokenType::ENDLOOP) && !isAtEnd()) {
        auto s = parseStatement();
        if (s) stmt.body.push_back(s);
        skipNewlines();
    }

    consume(TokenType::ENDLOOP, "Expected 'ENDLOOP'");
    return makeStmt(stmt);
}

StmtPtr Parser::parseWhile() {
    int line = peek().line;
    consume(TokenType::WHILE, "Expected 'WHILE'");

    WhileStmt stmt;
    stmt.sourceLine = line;

    stmt.condition = parseExpression();
    skipNewlines();

    while (!check(TokenType::ENDWHILE) && !isAtEnd()) {
        auto s = parseStatement();
        if (s) stmt.body.push_back(s);
        skipNewlines();
    }

    consume(TokenType::ENDWHILE, "Expected 'ENDWHILE'");
    return makeStmt(stmt);
}

// FOR counter = start TO end STEP increment ... ENDFOR
StmtPtr Parser::parseFor() {
    int line = peek().line;
    consume(TokenType::FOR, "Expected 'FOR'");

    ForStmt stmt;
    stmt.sourceLine = line;

    Token counter = consume(TokenType::IDENTIFIER, "Expected counter variable");
    stmt.counter = counter.lexeme;

    consume(TokenType::ASSIGN, "Expected '='");
    stmt.start = parseExpression();

    consume(TokenType::TO, "Expected 'TO'");
    stmt.end = parseExpression();

    // Optional STEP
    if (match({TokenType::STEP})) {
        stmt.step = parseExpression();
    }

    skipNewlines();

    while (!check(TokenType::ENDFOR) && !isAtEnd()) {
        auto s = parseStatement();
        if (s) stmt.body.push_back(s);
        skipNewlines();
    }

    consume(TokenType::ENDFOR, "Expected 'ENDFOR'");
    return makeStmt(stmt);
}

// REPEAT ... UNTIL condition
StmtPtr Parser::parseRepeat() {
    int line = peek().line;
    consume(TokenType::REPEAT, "Expected 'REPEAT'");

    RepeatStmt stmt;
    stmt.sourceLine = line;
    skipNewlines();

    while (!check(TokenType::UNTIL) && !isAtEnd()) {
        auto s = parseStatement();
        if (s) stmt.body.push_back(s);
        skipNewlines();
    }

    consume(TokenType::UNTIL, "Expected 'UNTIL'");
    stmt.condition = parseExpression();

    return makeStmt(stmt);
}

// SWITCH expr CASE val1[,val2] ... DEFAULT ... ENDSWITCH
StmtPtr Parser::parseSwitch() {
    int line = peek().line;
    consume(TokenType::SWITCH, "Expected 'SWITCH'");

    SwitchStmt stmt;
    stmt.sourceLine = line;

    stmt.selector = parseExpression();
    skipNewlines();

    while (!check(TokenType::ENDSWITCH) && !isAtEnd()) {
        if (match({TokenType::CASE})) {
            SwitchStmt::CaseClause clause;

            // Parse one or more comma-separated values
            clause.values.push_back(parseExpression());
            while (match({TokenType::COMMA})) {
                clause.values.push_back(parseExpression());
            }
            skipNewlines();

            // Parse body until next CASE, DEFAULT, or ENDSWITCH
            while (!check(TokenType::CASE) && !check(TokenType::DEFAULT) &&
                   !check(TokenType::ENDSWITCH) && !isAtEnd()) {
                auto s = parseStatement();
                if (s) clause.body.push_back(s);
                skipNewlines();
            }

            stmt.cases.push_back(std::move(clause));
        } else if (match({TokenType::DEFAULT})) {
            skipNewlines();
            while (!check(TokenType::ENDSWITCH) && !isAtEnd()) {
                auto s = parseStatement();
                if (s) stmt.defaultBody.push_back(s);
                skipNewlines();
            }
        } else {
            skipNewlines();
        }
    }

    consume(TokenType::ENDSWITCH, "Expected 'ENDSWITCH'");
    return makeStmt(stmt);
}

StmtPtr Parser::parseGoto() {
    int line = peek().line;
    consume(TokenType::GOTO, "Expected 'GOTO'");

    GotoStmt stmt;
    stmt.sourceLine = line;

    Token label = consume(TokenType::IDENTIFIER, "Expected label name");
    stmt.label = label.lexeme;

    return makeStmt(stmt);
}

StmtPtr Parser::parseHalt() {
    int line = peek().line;
    consume(TokenType::HALT, "Expected 'HALT'");

    HaltStmt stmt;
    stmt.sourceLine = line;

    return makeStmt(stmt);
}

StmtPtr Parser::parseContinue() {
    int line = peek().line;
    consume(TokenType::CONTINUE, "Expected 'CONTINUE'");

    ContinueStmt stmt;
    stmt.sourceLine = line;

    return makeStmt(stmt);
}

StmtPtr Parser::parseExit() {
    int line = peek().line;
    consume(TokenType::EXIT, "Expected 'EXIT'");

    ExitStmt stmt;
    stmt.sourceLine = line;

    return makeStmt(stmt);
}

// ============================================================================
// Expression Parsing - Recursive Descent
// ============================================================================

ExprPtr Parser::parseExpression() {
    return parseOr();
}

ExprPtr Parser::parseOr() {
    ExprPtr left = parseAnd();

    while (match({TokenType::OR, TokenType::EXOR, TokenType::B_OR, TokenType::B_EXOR})) {
        std::string op;
        switch (previous().type) {
            case TokenType::OR: op = "OR"; break;
            case TokenType::EXOR: op = "EXOR"; break;
            case TokenType::B_OR: op = "B_OR"; break;
            case TokenType::B_EXOR: op = "B_EXOR"; break;
            default: break;
        }
        ExprPtr right = parseAnd();
        BinaryExpr expr;
        expr.left = left;
        expr.op = op;
        expr.right = right;
        left = makeExpr(expr);
    }

    return left;
}

ExprPtr Parser::parseAnd() {
    ExprPtr left = parseEquality();

    while (match({TokenType::AND, TokenType::B_AND})) {
        std::string op = previous().type == TokenType::AND ? "AND" : "B_AND";
        ExprPtr right = parseEquality();
        BinaryExpr expr;
        expr.left = left;
        expr.op = op;
        expr.right = right;
        left = makeExpr(expr);
    }

    return left;
}

ExprPtr Parser::parseEquality() {
    ExprPtr left = parseComparison();

    while (match({TokenType::EQUAL, TokenType::NOT_EQUAL})) {
        std::string op = previous().type == TokenType::EQUAL ? "==" : "<>";
        ExprPtr right = parseComparison();
        BinaryExpr expr;
        expr.left = left;
        expr.op = op;
        expr.right = right;
        left = makeExpr(expr);
    }

    return left;
}

ExprPtr Parser::parseComparison() {
    ExprPtr left = parseTerm();

    while (match({TokenType::LESS, TokenType::GREATER, TokenType::LESS_EQ, TokenType::GREATER_EQ})) {
        std::string op;
        switch (previous().type) {
            case TokenType::LESS: op = "<"; break;
            case TokenType::GREATER: op = ">"; break;
            case TokenType::LESS_EQ: op = "<="; break;
            case TokenType::GREATER_EQ: op = ">="; break;
            default: break;
        }
        ExprPtr right = parseTerm();
        BinaryExpr expr;
        expr.left = left;
        expr.op = op;
        expr.right = right;
        left = makeExpr(expr);
    }

    return left;
}

ExprPtr Parser::parseTerm() {
    ExprPtr left = parseFactor();

    while (match({TokenType::PLUS, TokenType::MINUS})) {
        std::string op = previous().type == TokenType::PLUS ? "+" : "-";
        ExprPtr right = parseFactor();
        BinaryExpr expr;
        expr.left = left;
        expr.op = op;
        expr.right = right;
        left = makeExpr(expr);
    }

    return left;
}

ExprPtr Parser::parseFactor() {
    ExprPtr left = parseUnary();

    while (match({TokenType::STAR, TokenType::SLASH})) {
        std::string op = previous().type == TokenType::STAR ? "*" : "/";
        ExprPtr right = parseUnary();
        BinaryExpr expr;
        expr.left = left;
        expr.op = op;
        expr.right = right;
        left = makeExpr(expr);
    }

    return left;
}

ExprPtr Parser::parseUnary() {
    if (match({TokenType::MINUS, TokenType::NOT, TokenType::B_NOT})) {
        std::string op;
        switch (previous().type) {
            case TokenType::MINUS: op = "-"; break;
            case TokenType::NOT: op = "NOT"; break;
            case TokenType::B_NOT: op = "B_NOT"; break;
            default: break;
        }
        ExprPtr operand = parseUnary();
        UnaryExpr expr;
        expr.op = op;
        expr.operand = operand;
        return makeExpr(expr);
    }

    return parsePrimary();
}

ExprPtr Parser::parsePrimary() {
    if (match({TokenType::TRUE})) {
        LiteralExpr expr;
        expr.value = true;
        return makeExpr(expr);
    }

    if (match({TokenType::FALSE})) {
        LiteralExpr expr;
        expr.value = false;
        return makeExpr(expr);
    }

    if (match({TokenType::NUMBER})) {
        LiteralExpr expr;
        expr.value = std::get<double>(previous().literal);
        return makeExpr(expr);
    }

    if (match({TokenType::STRING})) {
        LiteralExpr expr;
        expr.value = std::get<std::string>(previous().literal);
        return makeExpr(expr);
    }

    // KRL aggregate: {X 10, Y 20, ...}
    if (check(TokenType::LBRACE)) {
        return parseAggregate();
    }

    // ENUM literal: #value
    if (match({TokenType::HASH})) {
        Token val = consume(TokenType::IDENTIFIER, "Expected enum value after '#'");
        LiteralExpr expr;
        expr.value = std::string("#") + val.lexeme;
        return makeExpr(expr);
    }

    // System variable $IN[x], $OUT[x], $POS_ACT, etc.
    if (check(TokenType::DOLLAR)) {
        return parseSystemVar();
    }

    // Identifier (with optional array subscript for TOOL_DATA[1], BASE_DATA[0])
    if (match({TokenType::IDENTIFIER})) {
        std::string name = previous().lexeme;
        if (match({TokenType::LBRACKET})) {
            ExprPtr indexExpr = parseExpression();
            consume(TokenType::RBRACKET, "Expected ']'");
            // Encode as "NAME[index]" variable reference
            VariableExpr expr;
            // Store as composite name for TOOL_DATA[N], BASE_DATA[N]
            if (auto* lit = std::get_if<LiteralExpr>(indexExpr.get())) {
                if (auto* dval = std::get_if<double>(&lit->value)) {
                    expr.name = name + "[" + std::to_string(static_cast<int>(*dval)) + "]";
                } else if (auto* ival = std::get_if<int>(&lit->value)) {
                    expr.name = name + "[" + std::to_string(*ival) + "]";
                } else {
                    expr.name = name;
                }
            } else {
                expr.name = name;
            }
            return makeExpr(expr);
        }
        VariableExpr expr;
        expr.name = name;
        return makeExpr(expr);
    }

    if (match({TokenType::HOME})) {
        PointExpr expr;
        expr.name = "HOME";
        return makeExpr(expr);
    }

    // Grouped expression
    if (match({TokenType::LPAREN})) {
        ExprPtr expr = parseExpression();
        consume(TokenType::RPAREN, "Expected ')'");
        return expr;
    }

    error("Expected expression");
    throw std::runtime_error("Expected expression");
}

ExprPtr Parser::parsePoint(const std::string& name) {
    PointExpr expr;
    expr.name = name;
    return makeExpr(expr);
}

ExprPtr Parser::parseSystemVar() {
    consume(TokenType::DOLLAR, "Expected '$'");
    Token name = consume(TokenType::IDENTIFIER, "Expected system variable name");

    SystemVarExpr expr;
    expr.name = name.lexeme;

    // Handle dotted notation: $VEL.CP, $APO.CDIS
    if (match({TokenType::DOT})) {
        Token subField = consume(TokenType::IDENTIFIER, "Expected sub-field after '.'");
        expr.subField = subField.lexeme;
    }

    if (match({TokenType::LBRACKET})) {
        expr.index = parseExpression();
        consume(TokenType::RBRACKET, "Expected ']'");
    }

    return makeExpr(expr);
}

// Parse KRL aggregate: {X 10, Y 20, Z 30, A 0, B 90, C 0}
// Also supports typed: {E6POS: X 10, Y 20, ...}
ExprPtr Parser::parseAggregate() {
    consume(TokenType::LBRACE, "Expected '{'");

    AggregateExpr expr;

    // Check for optional type prefix: {E6POS: ...}
    if (check(TokenType::IDENTIFIER) || isKrlType()) {
        // Peek ahead to see if this is "TYPE:" pattern
        size_t saved = m_current;
        Token typeTok = advance();

        if (check(TokenType::COLON)) {
            advance(); // consume ':'
            expr.typeName = typeTok.lexeme;
        } else {
            // Not a type prefix, it's a field name - backtrack
            m_current = saved;
        }
    }

    // Parse fields: FIELD_NAME value, FIELD_NAME value, ...
    while (!check(TokenType::RBRACE) && !isAtEnd()) {
        std::string fieldName;

        // Field name (e.g., X, Y, Z, A, B, C, A1, A2, S, T, E1...)
        if (check(TokenType::IDENTIFIER) || isKrlType()) {
            fieldName = advance().lexeme;
        } else {
            error("Expected field name in aggregate");
            break;
        }

        // Field value
        ExprPtr fieldValue = parseExpression();
        expr.fields.push_back({fieldName, fieldValue});

        if (!match({TokenType::COMMA})) break;
    }

    consume(TokenType::RBRACE, "Expected '}'");
    return makeExpr(expr);
}

} // namespace interpreter
} // namespace robot_controller
