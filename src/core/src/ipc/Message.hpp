/**
 * @file Message.hpp
 * @brief IPC Message structure and serialization
 */

#pragma once

#include <string>
#include <chrono>
#include <nlohmann/json.hpp>
#include "MessageTypes.hpp"

namespace robot_controller {
namespace ipc {

using json = nlohmann::json;

/**
 * IPC Message structure
 *
 * JSON Format:
 * {
 *   "type": "MESSAGE_TYPE",
 *   "id": "uuid-string",
 *   "timestamp": 1234567890123,
 *   "payload": { ... }
 * }
 */
struct Message {
    MessageType type = MessageType::UNKNOWN;
    std::string id;
    int64_t timestamp = 0;
    json payload;

    /**
     * Create a new message with auto-generated ID and timestamp
     */
    static Message create(MessageType type, const json& payload = {}) {
        Message msg;
        msg.type = type;
        msg.id = generateId();
        msg.timestamp = currentTimestamp();
        msg.payload = payload;
        return msg;
    }

    /**
     * Create a response message (keeps same ID)
     */
    static Message createResponse(const Message& request, MessageType responseType, const json& payload = {}) {
        Message msg;
        msg.type = responseType;
        msg.id = request.id;  // Keep same ID for request-response correlation
        msg.timestamp = currentTimestamp();
        msg.payload = payload;
        return msg;
    }

    /**
     * Serialize to JSON string
     */
    std::string serialize() const {
        json j;
        j["type"] = messageTypeToString(type);
        j["id"] = id;
        j["timestamp"] = timestamp;
        j["payload"] = payload;
        return j.dump();
    }

    /**
     * Deserialize from JSON string
     */
    static Message deserialize(const std::string& jsonStr) {
        Message msg;
        try {
            json j = json::parse(jsonStr);
            msg.type = stringToMessageType(j.value("type", "UNKNOWN"));
            msg.id = j.value("id", "");
            msg.timestamp = j.value("timestamp", 0LL);
            msg.payload = j.value("payload", json::object());
        } catch (const json::exception& e) {
            msg.type = MessageType::ERROR;
            msg.payload = {{"error", "Failed to parse message"}, {"details", e.what()}};
        }
        return msg;
    }

    /**
     * Check if message is valid
     */
    bool isValid() const {
        return type != MessageType::UNKNOWN && !id.empty();
    }

private:
    static std::string generateId() {
        // Simple UUID-like ID generation
        static uint64_t counter = 0;
        auto now = std::chrono::high_resolution_clock::now();
        auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(
            now.time_since_epoch()).count();

        char buf[64];
        snprintf(buf, sizeof(buf), "%016llx-%08llx",
                 static_cast<unsigned long long>(nanos),
                 static_cast<unsigned long long>(++counter));
        return std::string(buf);
    }

    static int64_t currentTimestamp() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
    }
};

// JSON serialization support for nlohmann::json
inline void to_json(json& j, const Message& msg) {
    j = json{
        {"type", messageTypeToString(msg.type)},
        {"id", msg.id},
        {"timestamp", msg.timestamp},
        {"payload", msg.payload}
    };
}

inline void from_json(const json& j, Message& msg) {
    msg.type = stringToMessageType(j.value("type", "UNKNOWN"));
    msg.id = j.value("id", "");
    msg.timestamp = j.value("timestamp", 0LL);
    msg.payload = j.value("payload", json::object());
}

} // namespace ipc
} // namespace robot_controller
