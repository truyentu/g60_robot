/**
 * @file Lexer.cpp
 * @brief Lexer implementation for KRL (KUKA Robot Language)
 *
 * Strict KUKA KRL syntax based on KSS 8.x specifications.
 * - Case-insensitive keyword matching
 * - Comments start with ; (semicolon)
 * - Assignment uses = (not :=)
 * - System variables start with $
 */

#include "Lexer.hpp"
#include <cctype>
#include <algorithm>

namespace robot_controller {
namespace interpreter {

// KRL keyword map (all uppercase for case-insensitive matching)
const std::unordered_map<std::string, TokenType> Lexer::s_keywords = {
    // Program structure
    {"DEF", TokenType::DEF},
    {"END", TokenType::END},
    {"DEFDAT", TokenType::DEFDAT},
    {"ENDDAT", TokenType::ENDDAT},
    {"PUBLIC", TokenType::PUBLIC},
    {"GLOBAL", TokenType::GLOBAL},

    // Declarations
    {"DECL", TokenType::DECL},
    {"CONST", TokenType::CONST},
    {"STRUC", TokenType::STRUC},
    {"ENUM", TokenType::ENUM},

    // Data types
    {"INT", TokenType::INT},
    {"REAL", TokenType::REAL},
    {"BOOL", TokenType::BOOL},
    {"CHAR", TokenType::CHAR},
    {"FRAME", TokenType::FRAME},
    {"POS", TokenType::POS},
    {"E6POS", TokenType::E6POS},
    {"E6AXIS", TokenType::E6AXIS},
    {"AXIS", TokenType::AXIS},

    // Control flow
    {"IF", TokenType::IF},
    {"THEN", TokenType::THEN},
    {"ELSE", TokenType::ELSE},
    {"ENDIF", TokenType::ENDIF},
    {"FOR", TokenType::FOR},
    {"TO", TokenType::TO},
    {"STEP", TokenType::STEP},
    {"ENDFOR", TokenType::ENDFOR},
    {"WHILE", TokenType::WHILE},
    {"ENDWHILE", TokenType::ENDWHILE},
    {"LOOP", TokenType::LOOP},
    {"ENDLOOP", TokenType::ENDLOOP},
    {"REPEAT", TokenType::REPEAT},
    {"UNTIL", TokenType::UNTIL},
    {"SWITCH", TokenType::SWITCH},
    {"CASE", TokenType::CASE},
    {"DEFAULT", TokenType::DEFAULT},
    {"ENDSWITCH", TokenType::ENDSWITCH},

    // Jump / flow control
    {"GOTO", TokenType::GOTO},
    {"HALT", TokenType::HALT},
    {"CONTINUE", TokenType::CONTINUE},
    {"EXIT", TokenType::EXIT},
    {"WAIT", TokenType::WAIT},
    {"SEC", TokenType::SEC},

    // Boolean literals
    {"TRUE", TokenType::TRUE},
    {"FALSE", TokenType::FALSE},

    // Motion commands
    {"PTP", TokenType::PTP},
    {"LIN", TokenType::LIN},
    {"CIRC", TokenType::CIRC},
    {"PTP_REL", TokenType::PTP_REL},
    {"LIN_REL", TokenType::LIN_REL},
    {"CIRC_REL", TokenType::CIRC_REL},
    {"HOME", TokenType::HOME},

    // Approximation keywords
    {"C_PTP", TokenType::C_PTP},
    {"C_DIS", TokenType::C_DIS},
    {"C_VEL", TokenType::C_VEL},
    {"C_ORI", TokenType::C_ORI},

    // Logical operators (KRL keywords)
    {"AND", TokenType::AND},
    {"OR", TokenType::OR},
    {"NOT", TokenType::NOT},
    {"EXOR", TokenType::EXOR},

    // Bitwise operators (KRL keywords)
    {"B_AND", TokenType::B_AND},
    {"B_OR", TokenType::B_OR},
    {"B_NOT", TokenType::B_NOT},
    {"B_EXOR", TokenType::B_EXOR}
};

Lexer::Lexer(const std::string& source)
    : m_source(source) {}

std::vector<Token> Lexer::tokenize() {
    while (!isAtEnd()) {
        m_start = m_current;
        scanToken();
    }

    m_tokens.push_back({TokenType::END_OF_FILE, "", std::monostate{}, m_line, m_column});
    return m_tokens;
}

bool Lexer::isAtEnd() const {
    return m_current >= m_source.length();
}

char Lexer::advance() {
    char c = m_source[m_current++];
    m_column++;
    return c;
}

char Lexer::peek() const {
    if (isAtEnd()) return '\0';
    return m_source[m_current];
}

char Lexer::peekNext() const {
    if (m_current + 1 >= m_source.length()) return '\0';
    return m_source[m_current + 1];
}

bool Lexer::match(char expected) {
    if (isAtEnd()) return false;
    if (m_source[m_current] != expected) return false;
    m_current++;
    m_column++;
    return true;
}

void Lexer::addToken(TokenType type) {
    addToken(type, std::monostate{});
}

void Lexer::addToken(TokenType type, std::variant<std::monostate, double, int, bool, std::string> literal) {
    std::string lexeme = m_source.substr(m_start, m_current - m_start);
    m_tokens.push_back({type, lexeme, literal, m_line, static_cast<int>(m_start - m_source.rfind('\n', m_start))});
}

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
        case '.': addToken(TokenType::DOT); break;
        case ':':
            addToken(TokenType::COLON);
            break;
        case '+': addToken(TokenType::PLUS); break;
        case '-': addToken(TokenType::MINUS); break;
        case '*': addToken(TokenType::STAR); break;
        case '/': addToken(TokenType::SLASH); break;
        case '%': addToken(TokenType::PERCENT); break;
        case '$': addToken(TokenType::DOLLAR); break;
        case '#': addToken(TokenType::HASH); break;

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

        // Semicolon: KRL comment delimiter
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
            break;

        default:
            if (std::isdigit(c)) {
                scanNumber();
            } else if (std::isalpha(c) || c == '_') {
                scanIdentifier();
            } else {
                error("Unexpected character: " + std::string(1, c));
            }
            break;
    }
}

void Lexer::scanNumber() {
    while (std::isdigit(peek())) advance();

    // Decimal part
    if (peek() == '.' && std::isdigit(peekNext())) {
        advance();  // consume '.'
        while (std::isdigit(peek())) advance();
    }

    // Scientific notation (e.g., 9E9, 1.5e-3)
    if (peek() == 'E' || peek() == 'e') {
        advance();
        if (peek() == '+' || peek() == '-') advance();
        if (!std::isdigit(peek())) {
            error("Expected digit after exponent");
            return;
        }
        while (std::isdigit(peek())) advance();
    }

    std::string lexeme = m_source.substr(m_start, m_current - m_start);
    double value = std::stod(lexeme);
    addToken(TokenType::NUMBER, value);
}

void Lexer::scanString() {
    while (peek() != '"' && !isAtEnd()) {
        if (peek() == '\n') {
            m_line++;
            m_column = 1;
        }
        advance();
    }

    if (isAtEnd()) {
        error("Unterminated string");
        return;
    }

    advance();  // closing "

    std::string value = m_source.substr(m_start + 1, m_current - m_start - 2);
    addToken(TokenType::STRING, value);
}

void Lexer::scanIdentifier() {
    while (std::isalnum(peek()) || peek() == '_') advance();

    std::string text = m_source.substr(m_start, m_current - m_start);

    // KRL is case-insensitive: normalize to uppercase for keyword matching
    std::string upper = text;
    std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);

    auto it = s_keywords.find(upper);
    if (it != s_keywords.end()) {
        if (it->second == TokenType::TRUE) {
            addToken(TokenType::TRUE, true);
        } else if (it->second == TokenType::FALSE) {
            addToken(TokenType::FALSE, false);
        } else {
            addToken(it->second);
        }
    } else {
        addToken(TokenType::IDENTIFIER, text);
    }
}

void Lexer::scanComment() {
    // KRL comment: ; until end of line
    while (peek() != '\n' && !isAtEnd()) {
        advance();
    }
    // Skip comment tokens (don't add to token stream)
}

void Lexer::error(const std::string& message) {
    std::string err = "Line " + std::to_string(m_line) + ": " + message;
    m_errors.push_back(err);
    addToken(TokenType::ERROR);
}

} // namespace interpreter
} // namespace robot_controller
