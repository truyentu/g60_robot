#pragma once

/**
 * @file Lexer.hpp
 * @brief Lexer for RPL (Robot Programming Language)
 *
 * Part of Phase 8: Virtual Simulation (IMPL_P8_02)
 */

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
