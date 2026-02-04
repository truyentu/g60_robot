#pragma once

/**
 * @file Token.hpp
 * @brief Token definitions for RPL (Robot Programming Language)
 *
 * Part of Phase 8: Virtual Simulation (IMPL_P8_02)
 */

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

inline std::string tokenTypeToString(TokenType type) {
    switch (type) {
        case TokenType::NUMBER: return "NUMBER";
        case TokenType::STRING: return "STRING";
        case TokenType::IDENTIFIER: return "IDENTIFIER";
        case TokenType::DEF: return "DEF";
        case TokenType::END: return "END";
        case TokenType::DECL: return "DECL";
        case TokenType::REAL: return "REAL";
        case TokenType::INT: return "INT";
        case TokenType::BOOL: return "BOOL";
        case TokenType::IF: return "IF";
        case TokenType::THEN: return "THEN";
        case TokenType::ELSE: return "ELSE";
        case TokenType::ENDIF: return "ENDIF";
        case TokenType::LOOP: return "LOOP";
        case TokenType::ENDLOOP: return "ENDLOOP";
        case TokenType::WHILE: return "WHILE";
        case TokenType::ENDWHILE: return "ENDWHILE";
        case TokenType::WAIT: return "WAIT";
        case TokenType::SEC: return "SEC";
        case TokenType::FOR: return "FOR";
        case TokenType::TRUE: return "TRUE";
        case TokenType::FALSE: return "FALSE";
        case TokenType::PTP: return "PTP";
        case TokenType::LIN: return "LIN";
        case TokenType::CIRC: return "CIRC";
        case TokenType::VEL: return "VEL";
        case TokenType::ACC: return "ACC";
        case TokenType::CONT: return "CONT";
        case TokenType::HOME: return "HOME";
        case TokenType::PLUS: return "PLUS";
        case TokenType::MINUS: return "MINUS";
        case TokenType::STAR: return "STAR";
        case TokenType::SLASH: return "SLASH";
        case TokenType::EQUAL: return "EQUAL";
        case TokenType::NOT_EQUAL: return "NOT_EQUAL";
        case TokenType::LESS: return "LESS";
        case TokenType::GREATER: return "GREATER";
        case TokenType::LESS_EQ: return "LESS_EQ";
        case TokenType::GREATER_EQ: return "GREATER_EQ";
        case TokenType::ASSIGN: return "ASSIGN";
        case TokenType::AND: return "AND";
        case TokenType::OR: return "OR";
        case TokenType::NOT: return "NOT";
        case TokenType::LPAREN: return "LPAREN";
        case TokenType::RPAREN: return "RPAREN";
        case TokenType::LBRACE: return "LBRACE";
        case TokenType::RBRACE: return "RBRACE";
        case TokenType::LBRACKET: return "LBRACKET";
        case TokenType::RBRACKET: return "RBRACKET";
        case TokenType::COMMA: return "COMMA";
        case TokenType::COLON: return "COLON";
        case TokenType::PERCENT: return "PERCENT";
        case TokenType::DOLLAR: return "DOLLAR";
        case TokenType::NEWLINE: return "NEWLINE";
        case TokenType::COMMENT: return "COMMENT";
        case TokenType::END_OF_FILE: return "EOF";
        case TokenType::ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

} // namespace interpreter
} // namespace robot_controller
