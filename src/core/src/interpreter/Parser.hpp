#pragma once

/**
 * @file Parser.hpp
 * @brief Parser for RPL (Robot Programming Language)
 *
 * Part of Phase 8: Virtual Simulation (IMPL_P8_02)
 */

#include "Token.hpp"
#include "AST.hpp"
#include <vector>
#include <optional>
#include <initializer_list>

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
    void skipNewlines();

    // Grammar rules
    ProgramStmt parseProgram();
    StmtPtr parseStatement();
    StmtPtr parseDeclaration();
    StmtPtr parseAssignment(const std::string& name);
    StmtPtr parseSystemAssignment();
    StmtPtr parseMotion(TokenType motionType);
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
    ExprPtr parsePoint(const std::string& name);
    ExprPtr parseSystemVar();
};

} // namespace interpreter
} // namespace robot_controller
