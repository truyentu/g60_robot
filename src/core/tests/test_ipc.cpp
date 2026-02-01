/**
 * @file test_ipc.cpp
 * @brief IPC Server tests
 */

#include <gtest/gtest.h>
#include <thread>
#include <chrono>
#include "ipc/IpcServer.hpp"
#include "ipc/Message.hpp"
#include "logging/Logger.hpp"

using namespace robot_controller;
using namespace robot_controller::ipc;

class IpcServerTest : public ::testing::Test {
protected:
    void SetUp() override {
        Logger::init("test_ipc.log", "debug");
    }

    void TearDown() override {
        // Cleanup
    }
};

TEST_F(IpcServerTest, CreateInstance) {
    IpcServer server("tcp://*:15555", "tcp://*:15556");
    SUCCEED();
}

TEST_F(IpcServerTest, StartStop) {
    IpcServer server("tcp://*:15557", "tcp://*:15558");

    EXPECT_FALSE(server.isRunning());

    EXPECT_TRUE(server.start());
    EXPECT_TRUE(server.isRunning());

    // Let it run briefly
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    server.stop();
    EXPECT_FALSE(server.isRunning());
}

TEST_F(IpcServerTest, MessageSerialization) {
    Message msg = Message::create(MessageType::PING, {{"test", "value"}});

    EXPECT_EQ(msg.type, MessageType::PING);
    EXPECT_FALSE(msg.id.empty());
    EXPECT_GT(msg.timestamp, 0);

    std::string json = msg.serialize();
    EXPECT_FALSE(json.empty());
    EXPECT_NE(json.find("PING"), std::string::npos);
}

TEST_F(IpcServerTest, MessageDeserialization) {
    std::string json = R"({
        "type": "PING",
        "id": "test-123",
        "timestamp": 1234567890,
        "payload": {"key": "value"}
    })";

    Message msg = Message::deserialize(json);

    EXPECT_EQ(msg.type, MessageType::PING);
    EXPECT_EQ(msg.id, "test-123");
    EXPECT_EQ(msg.timestamp, 1234567890);
    EXPECT_TRUE(msg.isValid());
}

TEST_F(IpcServerTest, InvalidMessageDeserialization) {
    std::string invalid_json = "not valid json";

    Message msg = Message::deserialize(invalid_json);

    EXPECT_EQ(msg.type, MessageType::ERROR);
}

TEST_F(IpcServerTest, RegisterHandler) {
    IpcServer server("tcp://*:15559", "tcp://*:15560");

    bool handler_called = false;
    server.registerHandler(MessageType::GET_STATUS, [&handler_called](const Message& req) {
        handler_called = true;
        return nlohmann::json{{"state", "IDLE"}};
    });

    SUCCEED();  // Handler registration doesn't throw
}
