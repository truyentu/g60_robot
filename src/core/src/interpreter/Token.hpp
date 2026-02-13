#pragma once

/**
 * @file Token.hpp
 * @brief Token definitions for KRL (KUKA Robot Language) interpreter
 *
 * Strict KUKA KRL syntax based on KSS 8.x specifications.
 */

#include <string>
#include <variant>

namespace robot_controller {
namespace interpreter {

enum class TokenType {
    // Literals
    NUMBER,         // 123, 45.67, 9E9
    STRING,         // "hello"
    IDENTIFIER,     // variable names, point names

    // Program Structure
    DEF, END,
    DEFDAT, ENDDAT,
    PUBLIC, GLOBAL,

    // Declarations
    DECL, CONST,
    STRUC, ENUM,

    // Data Types
    INT, REAL, BOOL, CHAR,
    FRAME, POS, E6POS, E6AXIS, AXIS,

    // Control Flow
    IF, THEN, ELSE, ENDIF,
    FOR, TO, STEP, ENDFOR,
    WHILE, ENDWHILE,
    LOOP, ENDLOOP,
    REPEAT, UNTIL,
    SWITCH, CASE, DEFAULT, ENDSWITCH,

    // Jump / Flow Control
    GOTO, HALT, CONTINUE, EXIT,
    WAIT, SEC,

    // Boolean Literals
    TRUE, FALSE,

    // Motion Commands
    PTP, LIN, CIRC,
    PTP_REL, LIN_REL, CIRC_REL,
    HOME,

    // Approximation
    C_PTP, C_DIS, C_VEL, C_ORI,

    // Arithmetic Operators
    PLUS, MINUS, STAR, SLASH,

    // Comparison Operators
    EQUAL,          // ==
    NOT_EQUAL,      // <>
    LESS, GREATER, LESS_EQ, GREATER_EQ,

    // Assignment
    ASSIGN,         // =

    // Logical Operators
    AND, OR, NOT, EXOR,

    // Bitwise Operators
    B_AND, B_OR, B_NOT, B_EXOR,

    // Punctuation
    LPAREN, RPAREN,         // ( )
    LBRACE, RBRACE,         // { }
    LBRACKET, RBRACKET,     // [ ]
    COMMA,                  // ,
    DOT,                    // . (for dotted system variables: $VEL.CP, $APO.CDIS)
    COLON,                  // : (also geometric operator for frame concatenation)
    PERCENT,                // %
    DOLLAR,                 // $
    HASH,                   // #  (for ENUM literal #value)

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
        case TokenType::DEFDAT: return "DEFDAT";
        case TokenType::ENDDAT: return "ENDDAT";
        case TokenType::PUBLIC: return "PUBLIC";
        case TokenType::GLOBAL: return "GLOBAL";
        case TokenType::DECL: return "DECL";
        case TokenType::CONST: return "CONST";
        case TokenType::STRUC: return "STRUC";
        case TokenType::ENUM: return "ENUM";
        case TokenType::INT: return "INT";
        case TokenType::REAL: return "REAL";
        case TokenType::BOOL: return "BOOL";
        case TokenType::CHAR: return "CHAR";
        case TokenType::FRAME: return "FRAME";
        case TokenType::POS: return "POS";
        case TokenType::E6POS: return "E6POS";
        case TokenType::E6AXIS: return "E6AXIS";
        case TokenType::AXIS: return "AXIS";
        case TokenType::IF: return "IF";
        case TokenType::THEN: return "THEN";
        case TokenType::ELSE: return "ELSE";
        case TokenType::ENDIF: return "ENDIF";
        case TokenType::FOR: return "FOR";
        case TokenType::TO: return "TO";
        case TokenType::STEP: return "STEP";
        case TokenType::ENDFOR: return "ENDFOR";
        case TokenType::WHILE: return "WHILE";
        case TokenType::ENDWHILE: return "ENDWHILE";
        case TokenType::LOOP: return "LOOP";
        case TokenType::ENDLOOP: return "ENDLOOP";
        case TokenType::REPEAT: return "REPEAT";
        case TokenType::UNTIL: return "UNTIL";
        case TokenType::SWITCH: return "SWITCH";
        case TokenType::CASE: return "CASE";
        case TokenType::DEFAULT: return "DEFAULT";
        case TokenType::ENDSWITCH: return "ENDSWITCH";
        case TokenType::GOTO: return "GOTO";
        case TokenType::HALT: return "HALT";
        case TokenType::CONTINUE: return "CONTINUE";
        case TokenType::EXIT: return "EXIT";
        case TokenType::WAIT: return "WAIT";
        case TokenType::SEC: return "SEC";
        case TokenType::TRUE: return "TRUE";
        case TokenType::FALSE: return "FALSE";
        case TokenType::PTP: return "PTP";
        case TokenType::LIN: return "LIN";
        case TokenType::CIRC: return "CIRC";
        case TokenType::PTP_REL: return "PTP_REL";
        case TokenType::LIN_REL: return "LIN_REL";
        case TokenType::CIRC_REL: return "CIRC_REL";
        case TokenType::HOME: return "HOME";
        case TokenType::C_PTP: return "C_PTP";
        case TokenType::C_DIS: return "C_DIS";
        case TokenType::C_VEL: return "C_VEL";
        case TokenType::C_ORI: return "C_ORI";
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
        case TokenType::EXOR: return "EXOR";
        case TokenType::B_AND: return "B_AND";
        case TokenType::B_OR: return "B_OR";
        case TokenType::B_NOT: return "B_NOT";
        case TokenType::B_EXOR: return "B_EXOR";
        case TokenType::LPAREN: return "LPAREN";
        case TokenType::RPAREN: return "RPAREN";
        case TokenType::LBRACE: return "LBRACE";
        case TokenType::RBRACE: return "RBRACE";
        case TokenType::LBRACKET: return "LBRACKET";
        case TokenType::RBRACKET: return "RBRACKET";
        case TokenType::COMMA: return "COMMA";
        case TokenType::DOT: return "DOT";
        case TokenType::COLON: return "COLON";
        case TokenType::PERCENT: return "PERCENT";
        case TokenType::DOLLAR: return "DOLLAR";
        case TokenType::HASH: return "HASH";
        case TokenType::NEWLINE: return "NEWLINE";
        case TokenType::COMMENT: return "COMMENT";
        case TokenType::END_OF_FILE: return "EOF";
        case TokenType::ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

} // namespace interpreter
} // namespace robot_controller
