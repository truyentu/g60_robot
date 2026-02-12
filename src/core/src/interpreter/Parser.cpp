/**
 * @file Parser.cpp
 * @brief Parser implementation for RPL
 *
 * Part of Phase 8: Virtual Simulation (IMPL_P8_02)
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
        // Synchronize at statement boundaries
        if (previous().type == TokenType::NEWLINE) return;
        if (previous().type == TokenType::SEMICOLON) return;
        switch (peek().type) {
            case TokenType::DEF:
            case TokenType::IF:
            case TokenType::LOOP:
            case TokenType::WHILE:
            case TokenType::PTP:
            case TokenType::LIN:
            case TokenType::CIRC:
            case TokenType::MOVEJ:
            case TokenType::MOVEL:
            case TokenType::MOVEC:
            case TokenType::WAIT:
            case TokenType::DECL:
            case TokenType::CONST:
                return;
            default:
                advance();
        }
    }
}

void Parser::skipNewlines() {
    while (match({TokenType::NEWLINE})) {}
}

ProgramStmt Parser::parseProgram() {
    ProgramStmt program;

    skipNewlines();

    // Phase 1: Parse top-level CONST declarations (before DEF)
    while (check(TokenType::CONST) && !isAtEnd()) {
        try {
            auto constDecl = parseConstDeclaration();
            if (constDecl) {
                program.constDecls.push_back(constDecl);
            }
        } catch (const std::exception&) {
            synchronize();
        }
        skipNewlines();
    }

    // Phase 2: DEF name() ... END
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

StmtPtr Parser::parseStatement() {
    skipNewlines();

    // Skip standalone semicolons
    while (match({TokenType::SEMICOLON})) {
        skipNewlines();
    }

    if (check(TokenType::DECL)) {
        return parseDeclaration();
    }

    if (check(TokenType::CONST)) {
        auto stmt = parseConstDeclaration();
        consumeOptionalSemicolon();
        return stmt;
    }

    // KRL-style motion: PTP, LIN, CIRC
    if (check(TokenType::PTP) || check(TokenType::LIN) || check(TokenType::CIRC)) {
        TokenType type = peek().type;
        advance();
        auto stmt = parseMotion(type);
        consumeOptionalSemicolon();
        return stmt;
    }

    // RAPID-style motion: MoveJ, MoveL, MoveC
    if (check(TokenType::MOVEJ) || check(TokenType::MOVEL) || check(TokenType::MOVEC)) {
        TokenType type = peek().type;
        advance();
        auto stmt = parseRapidMotion(type);
        consumeOptionalSemicolon();
        return stmt;
    }

    if (check(TokenType::WAIT)) {
        auto stmt = parseWait();
        consumeOptionalSemicolon();
        return stmt;
    }

    if (check(TokenType::IF)) {
        return parseIf();
    }

    if (check(TokenType::LOOP)) {
        return parseLoop();
    }

    if (check(TokenType::WHILE)) {
        return parseWhile();
    }

    if (check(TokenType::DOLLAR)) {
        auto stmt = parseSystemAssignment();
        consumeOptionalSemicolon();
        return stmt;
    }

    if (check(TokenType::IDENTIFIER)) {
        Token name = advance();
        if (check(TokenType::ASSIGN)) {
            auto stmt = parseAssignment(name.lexeme);
            consumeOptionalSemicolon();
            return stmt;
        }
        if (check(TokenType::COLONASSIGN)) {
            // name := expr (RAPID-style assignment)
            advance(); // consume :=
            AssignStmt stmt;
            stmt.name = name.lexeme;
            stmt.value = parseExpression();
            consumeOptionalSemicolon();
            return makeStmt(stmt);
        }
        if (check(TokenType::LPAREN) || check(TokenType::SEMICOLON) || check(TokenType::NEWLINE) || isAtEnd()) {
            // Function call: ArcStart(...); or ArcEnd;
            auto stmt = parseFunctionCall(name.lexeme);
            consumeOptionalSemicolon();
            return stmt;
        }
        // Could be just a point reference or something else
        error("Unexpected identifier: " + name.lexeme);
        return nullptr;
    }

    if (check(TokenType::NEWLINE) || check(TokenType::SEMICOLON) || isAtEnd()) {
        return nullptr;
    }

    error("Unexpected token: " + peek().lexeme);
    return nullptr;
}

void Parser::consumeOptionalSemicolon() {
    match({TokenType::SEMICOLON});
}

StmtPtr Parser::parseDeclaration() {
    int line = peek().line;
    consume(TokenType::DECL, "Expected 'DECL'");

    DeclareStmt stmt;

    // Type
    if (match({TokenType::REAL})) {
        stmt.type = "REAL";
    } else if (match({TokenType::INT})) {
        stmt.type = "INT";
    } else if (match({TokenType::BOOL})) {
        stmt.type = "BOOL";
    } else {
        error("Expected type (REAL, INT, BOOL)");
        return nullptr;
    }

    // Name
    Token name = consume(TokenType::IDENTIFIER, "Expected variable name");
    stmt.name = name.lexeme;

    // Optional initializer (= or :=)
    if (match({TokenType::ASSIGN, TokenType::COLONASSIGN})) {
        stmt.initializer = parseExpression();
    }

    consumeOptionalSemicolon();
    return makeStmt(stmt);
}

StmtPtr Parser::parseConstDeclaration() {
    int line = peek().line;
    consume(TokenType::CONST, "Expected 'CONST'");

    ConstDeclStmt stmt;
    stmt.sourceLine = line;

    // Type (robtarget, or any identifier)
    if (match({TokenType::ROBTARGET})) {
        stmt.type = "robtarget";
    } else if (check(TokenType::IDENTIFIER)) {
        stmt.type = advance().lexeme;
    } else {
        error("Expected type after CONST");
        return nullptr;
    }

    // Name
    Token name = consume(TokenType::IDENTIFIER, "Expected variable name");
    stmt.name = name.lexeme;

    // := value
    if (!match({TokenType::COLONASSIGN})) {
        consume(TokenType::ASSIGN, "Expected ':=' or '='");
    }

    // Capture raw value using bracket counting for inline arrays [[...]]
    std::string rawValue;
    if (check(TokenType::LBRACKET)) {
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
            } else if (tok.type == TokenType::NEWLINE || tok.type == TokenType::SEMICOLON) {
                break;
            } else {
                rawValue += tok.lexeme;
                advance();
            }
        }
    } else {
        // Non-array value (just capture until semicolon/newline)
        while (!check(TokenType::SEMICOLON) && !check(TokenType::NEWLINE) && !isAtEnd()) {
            rawValue += peek().lexeme;
            advance();
        }
    }

    stmt.rawValue = rawValue;
    consumeOptionalSemicolon();
    return makeStmt(stmt);
}

StmtPtr Parser::parseFunctionCall(const std::string& name) {
    int line = previous().line;

    FunctionCallStmt stmt;
    stmt.name = name;
    stmt.sourceLine = line;

    // Optional argument list
    if (match({TokenType::LPAREN})) {
        // Parse arguments (possibly named: key:=value)
        while (!check(TokenType::RPAREN) && !isAtEnd()) {
            std::string argName;
            ExprPtr argValue;

            if (check(TokenType::IDENTIFIER)) {
                Token id = advance();
                if (match({TokenType::COLONASSIGN})) {
                    // Named argument: key := value
                    argName = id.lexeme;
                    argValue = parseExpression();
                } else {
                    // Positional argument
                    VariableExpr ve;
                    ve.name = id.lexeme;
                    argValue = makeExpr(ve);
                }
            } else {
                argValue = parseExpression();
            }

            stmt.args.push_back({argName, argValue});

            if (!match({TokenType::COMMA})) break;
        }
        consume(TokenType::RPAREN, "Expected ')'");
    }

    return makeStmt(stmt);
}

StmtPtr Parser::parseRapidMotion(TokenType motionType) {
    int line = previous().line;

    MotionStmt stmt;
    stmt.sourceLine = line;

    switch (motionType) {
        case TokenType::MOVEJ: stmt.type = "MoveJ"; break;
        case TokenType::MOVEL: stmt.type = "MoveL"; break;
        case TokenType::MOVEC: stmt.type = "MoveC"; break;
        default: break;
    }

    // RAPID motion: MoveL target, speed, zone, tool;
    // First arg: target point
    if (check(TokenType::HOME)) {
        advance();
        PointExpr point;
        point.name = "HOME";
        stmt.target = makeExpr(point);
    } else if (check(TokenType::IDENTIFIER)) {
        Token name = advance();
        PointExpr point;
        point.name = name.lexeme;
        stmt.target = makeExpr(point);
    } else {
        error("Expected point name");
        return nullptr;
    }

    // For MoveC, need auxiliary point before end point
    if (motionType == TokenType::MOVEC && check(TokenType::COMMA)) {
        advance(); // consume comma
        if (check(TokenType::IDENTIFIER)) {
            Token auxName = advance();
            PointExpr auxPoint;
            auxPoint.name = auxName.lexeme;
            stmt.auxPoint = makeExpr(auxPoint);
        }
    }

    // Remaining args: speed, zone, tool (comma separated identifiers)
    if (match({TokenType::COMMA})) {
        // Speed (v100, vmax, etc.)
        if (check(TokenType::IDENTIFIER) || check(TokenType::NUMBER)) {
            Token speedTok = advance();
            stmt.speedName = speedTok.lexeme;

            // Parse speed value from name (v100 → 100, vmax → 5000)
            std::string sn = stmt.speedName;
            std::transform(sn.begin(), sn.end(), sn.begin(), ::tolower);
            if (sn == "vmax") {
                stmt.velocity = 5000;
                stmt.velocityUnit = "mm/s";
            } else if (sn.size() > 1 && sn[0] == 'v') {
                try {
                    stmt.velocity = std::stod(sn.substr(1));
                    stmt.velocityUnit = "mm/s";
                } catch (...) {}
            }
        }

        if (match({TokenType::COMMA})) {
            // Zone (fine, z50, etc.)
            if (check(TokenType::IDENTIFIER) || check(TokenType::NUMBER)) {
                stmt.zoneName = advance().lexeme;
                std::string zn = stmt.zoneName;
                std::transform(zn.begin(), zn.end(), zn.begin(), ::tolower);
                if (zn == "fine") {
                    stmt.continuous = false;
                } else {
                    stmt.continuous = true;  // fly-by zone
                }
            }

            if (match({TokenType::COMMA})) {
                // Tool (tool0, etc.)
                if (check(TokenType::IDENTIFIER)) {
                    stmt.toolName = advance().lexeme;
                }
            }
        }
    }

    return makeStmt(stmt);
}

StmtPtr Parser::parseAssignment(const std::string& name) {
    consume(TokenType::ASSIGN, "Expected '='");

    AssignStmt stmt;
    stmt.name = name;
    stmt.value = parseExpression();

    return makeStmt(stmt);
}

StmtPtr Parser::parseSystemAssignment() {
    int line = peek().line;
    consume(TokenType::DOLLAR, "Expected '$'");

    Token varName = consume(TokenType::IDENTIFIER, "Expected system variable name");

    SystemAssignStmt stmt;
    stmt.name = varName.lexeme;

    // Index
    consume(TokenType::LBRACKET, "Expected '['");
    stmt.index = parseExpression();
    consume(TokenType::RBRACKET, "Expected ']'");

    consume(TokenType::ASSIGN, "Expected '='");
    stmt.value = parseExpression();

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
        default: break;
    }

    // Target point
    if (check(TokenType::HOME)) {
        advance();
        PointExpr point;
        point.name = "HOME";
        stmt.target = makeExpr(point);
    } else if (check(TokenType::IDENTIFIER)) {
        Token name = advance();
        stmt.target = parsePoint(name.lexeme);
    } else {
        error("Expected point name");
        return nullptr;
    }

    // For CIRC, need auxiliary point
    if (motionType == TokenType::CIRC && check(TokenType::IDENTIFIER)) {
        Token auxName = advance();
        stmt.auxPoint = parsePoint(auxName.lexeme);
    }

    // Optional parameters: VEL=100%, ACC=100, CONT
    while (check(TokenType::VEL) || check(TokenType::ACC) || check(TokenType::CONT)) {
        if (match({TokenType::VEL})) {
            consume(TokenType::ASSIGN, "Expected '='");
            Token val = consume(TokenType::NUMBER, "Expected velocity value");
            stmt.velocity = std::get<double>(val.literal);

            if (match({TokenType::PERCENT})) {
                stmt.velocityUnit = "percent";
            } else if (check(TokenType::IDENTIFIER)) {
                // Check for mm/s
                Token unit = advance();
                std::string u = unit.lexeme;
                std::transform(u.begin(), u.end(), u.begin(), ::tolower);
                if (u == "mm" || u == "mms") {
                    stmt.velocityUnit = "mm/s";
                }
            }
        }

        if (match({TokenType::ACC})) {
            consume(TokenType::ASSIGN, "Expected '='");
            Token val = consume(TokenType::NUMBER, "Expected acceleration value");
            stmt.acceleration = std::get<double>(val.literal);
        }

        if (match({TokenType::CONT})) {
            stmt.continuous = true;
        }
    }

    return makeStmt(stmt);
}

StmtPtr Parser::parseWait() {
    int line = peek().line;
    consume(TokenType::WAIT, "Expected 'WAIT'");

    WaitStmt stmt;
    stmt.sourceLine = line;

    if (match({TokenType::SEC})) {
        // WAIT SEC <time>
        stmt.type = WaitStmt::WaitType::TIME;
        stmt.value = parseExpression();
    } else if (match({TokenType::FOR})) {
        // WAIT FOR <condition>
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

    // Then branch
    while (!check(TokenType::ELSE) && !check(TokenType::ENDIF) && !isAtEnd()) {
        auto s = parseStatement();
        if (s) stmt.thenBranch.push_back(s);
        skipNewlines();
    }

    // Else branch
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

    // Optional count
    if (check(TokenType::NUMBER)) {
        stmt.count = parseExpression();
    }
    skipNewlines();

    // Body
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

    // Body
    while (!check(TokenType::ENDWHILE) && !isAtEnd()) {
        auto s = parseStatement();
        if (s) stmt.body.push_back(s);
        skipNewlines();
    }

    consume(TokenType::ENDWHILE, "Expected 'ENDWHILE'");
    return makeStmt(stmt);
}

// Expression parsing - recursive descent
ExprPtr Parser::parseExpression() {
    return parseOr();
}

ExprPtr Parser::parseOr() {
    ExprPtr left = parseAnd();

    while (match({TokenType::OR})) {
        ExprPtr right = parseAnd();
        BinaryExpr expr;
        expr.left = left;
        expr.op = "OR";
        expr.right = right;
        left = makeExpr(expr);
    }

    return left;
}

ExprPtr Parser::parseAnd() {
    ExprPtr left = parseEquality();

    while (match({TokenType::AND})) {
        ExprPtr right = parseEquality();
        BinaryExpr expr;
        expr.left = left;
        expr.op = "AND";
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
    if (match({TokenType::MINUS, TokenType::NOT})) {
        std::string op = previous().type == TokenType::MINUS ? "-" : "NOT";
        ExprPtr operand = parseUnary();
        UnaryExpr expr;
        expr.op = op;
        expr.operand = operand;
        return makeExpr(expr);
    }

    return parsePrimary();
}

ExprPtr Parser::parsePrimary() {
    // Boolean literals
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

    // Number
    if (match({TokenType::NUMBER})) {
        LiteralExpr expr;
        expr.value = std::get<double>(previous().literal);
        return makeExpr(expr);
    }

    // String
    if (match({TokenType::STRING})) {
        LiteralExpr expr;
        expr.value = std::get<std::string>(previous().literal);
        return makeExpr(expr);
    }

    // System variable $IN[x], $OUT[x]
    if (check(TokenType::DOLLAR)) {
        return parseSystemVar();
    }

    // Identifier (variable or point)
    if (match({TokenType::IDENTIFIER})) {
        std::string name = previous().lexeme;
        VariableExpr expr;
        expr.name = name;
        return makeExpr(expr);
    }

    // HOME
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

    if (match({TokenType::LBRACKET})) {
        expr.index = parseExpression();
        consume(TokenType::RBRACKET, "Expected ']'");
    }

    return makeExpr(expr);
}

} // namespace interpreter
} // namespace robot_controller
