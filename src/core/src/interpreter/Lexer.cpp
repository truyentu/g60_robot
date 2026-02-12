/**
 * @file Lexer.cpp
 * @brief Lexer implementation for RPL
 *
 * Part of Phase 8: Virtual Simulation (IMPL_P8_02)
 */

#include "Lexer.hpp"
#include <cctype>
#include <algorithm>

namespace robot_controller {
namespace interpreter {

// Keyword map (case-insensitive matching)
const std::unordered_map<std::string, TokenType> Lexer::s_keywords = {
    {"DEF", TokenType::DEF},
    {"END", TokenType::END},
    {"DECL", TokenType::DECL},
    {"CONST", TokenType::CONST},
    {"ROBTARGET", TokenType::ROBTARGET},
    {"REAL", TokenType::REAL},
    {"INT", TokenType::INT},
    {"BOOL", TokenType::BOOL},
    {"IF", TokenType::IF},
    {"THEN", TokenType::THEN},
    {"ELSE", TokenType::ELSE},
    {"ENDIF", TokenType::ENDIF},
    {"LOOP", TokenType::LOOP},
    {"ENDLOOP", TokenType::ENDLOOP},
    {"WHILE", TokenType::WHILE},
    {"ENDWHILE", TokenType::ENDWHILE},
    {"WAIT", TokenType::WAIT},
    {"SEC", TokenType::SEC},
    {"FOR", TokenType::FOR},
    {"TRUE", TokenType::TRUE},
    {"FALSE", TokenType::FALSE},
    {"PTP", TokenType::PTP},
    {"LIN", TokenType::LIN},
    {"CIRC", TokenType::CIRC},
    {"MOVEJ", TokenType::MOVEJ},
    {"MOVEL", TokenType::MOVEL},
    {"MOVEC", TokenType::MOVEC},
    {"VEL", TokenType::VEL},
    {"ACC", TokenType::ACC},
    {"CONT", TokenType::CONT},
    {"HOME", TokenType::HOME},
    {"AND", TokenType::AND},
    {"OR", TokenType::OR},
    {"NOT", TokenType::NOT}
};

Lexer::Lexer(const std::string& source)
    : m_source(source) {}

std::vector<Token> Lexer::tokenize() {
    while (!isAtEnd()) {
        m_start = m_current;
        scanToken();
    }

    // Add EOF token
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
        case ':':
            addToken(match('=') ? TokenType::COLONASSIGN : TokenType::COLON);
            break;
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
            addToken(TokenType::SEMICOLON);
            break;

        case '!':
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

    // Look for decimal
    if (peek() == '.' && std::isdigit(peekNext())) {
        advance();  // consume '.'
        while (std::isdigit(peek())) advance();
    }

    // Look for scientific notation (e.g., 9E9, 1.5e-3)
    if (peek() == 'E' || peek() == 'e') {
        advance();  // consume 'E'/'e'
        if (peek() == '+' || peek() == '-') advance();  // optional sign
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

    // Trim quotes
    std::string value = m_source.substr(m_start + 1, m_current - m_start - 2);
    addToken(TokenType::STRING, value);
}

void Lexer::scanIdentifier() {
    while (std::isalnum(peek()) || peek() == '_') advance();

    std::string text = m_source.substr(m_start, m_current - m_start);

    // Convert to uppercase for keyword matching
    std::string upper = text;
    std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);

    auto it = s_keywords.find(upper);
    if (it != s_keywords.end()) {
        // Special handling for TRUE/FALSE
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
    // Comment runs until end of line
    while (peek() != '\n' && !isAtEnd()) {
        advance();
    }
    // Optionally add comment token (we skip it for now)
    // addToken(TokenType::COMMENT);
}

void Lexer::error(const std::string& message) {
    std::string err = "Line " + std::to_string(m_line) + ": " + message;
    m_errors.push_back(err);
    addToken(TokenType::ERROR);
}

} // namespace interpreter
} // namespace robot_controller
