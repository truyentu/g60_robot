/**
 * @file test_state_machine.cpp
 * @brief State machine tests
 */

#include <gtest/gtest.h>
#include "state/StateMachine.hpp"
#include "logging/Logger.hpp"

using namespace robot_controller::state;

class StateMachineTest : public ::testing::Test {
protected:
    void SetUp() override {
        robot_controller::Logger::init("test_state_machine.log", "debug");
        sm = std::make_unique<StateMachine>();
    }

    std::unique_ptr<StateMachine> sm;
};

// Initial state tests
TEST_F(StateMachineTest, InitialState) {
    EXPECT_EQ(sm->currentState(), RobotState::INIT);
    EXPECT_EQ(sm->currentMode(), RobotMode::MANUAL);
    EXPECT_FALSE(sm->isEnabled());
    EXPECT_FALSE(sm->isHomed());
}

TEST_F(StateMachineTest, InitToIdle) {
    EXPECT_TRUE(sm->processEvent(StateEvent::INIT_COMPLETE));
    EXPECT_EQ(sm->currentState(), RobotState::IDLE);
}

// Enable/Disable tests
TEST_F(StateMachineTest, EnableFromIdle) {
    sm->processEvent(StateEvent::INIT_COMPLETE);
    EXPECT_TRUE(sm->enable());
    EXPECT_TRUE(sm->isEnabled());
}

TEST_F(StateMachineTest, DisableFromEnabled) {
    sm->processEvent(StateEvent::INIT_COMPLETE);
    sm->enable();
    EXPECT_TRUE(sm->disable());
    EXPECT_FALSE(sm->isEnabled());
}

// Homing tests
TEST_F(StateMachineTest, HomingSequence) {
    sm->processEvent(StateEvent::INIT_COMPLETE);
    sm->enable();

    // Start homing
    EXPECT_TRUE(sm->processEvent(StateEvent::HOME_REQUEST));
    EXPECT_EQ(sm->currentState(), RobotState::HOMING);

    // Complete homing
    EXPECT_TRUE(sm->processEvent(StateEvent::HOME_COMPLETE));
    EXPECT_EQ(sm->currentState(), RobotState::READY);
    EXPECT_TRUE(sm->isHomed());
}

TEST_F(StateMachineTest, CannotHomeWithoutEnable) {
    sm->processEvent(StateEvent::INIT_COMPLETE);
    // Not enabled
    EXPECT_FALSE(sm->processEvent(StateEvent::HOME_REQUEST));
    EXPECT_EQ(sm->currentState(), RobotState::IDLE);
}

// Motion tests
TEST_F(StateMachineTest, MotionFromReady) {
    sm->processEvent(StateEvent::INIT_COMPLETE);
    sm->enable();
    sm->processEvent(StateEvent::HOME_REQUEST);
    sm->processEvent(StateEvent::HOME_COMPLETE);

    EXPECT_TRUE(sm->processEvent(StateEvent::MOTION_START));
    EXPECT_EQ(sm->currentState(), RobotState::MOVING);
    EXPECT_TRUE(sm->isMoving());

    EXPECT_TRUE(sm->processEvent(StateEvent::MOTION_COMPLETE));
    EXPECT_EQ(sm->currentState(), RobotState::READY);
}

TEST_F(StateMachineTest, CanJogInManualMode) {
    sm->processEvent(StateEvent::INIT_COMPLETE);
    sm->enable();
    sm->processEvent(StateEvent::HOME_REQUEST);
    sm->processEvent(StateEvent::HOME_COMPLETE);

    EXPECT_TRUE(sm->canJog());
}

// Mode tests
TEST_F(StateMachineTest, ChangeModeWhenReady) {
    sm->processEvent(StateEvent::INIT_COMPLETE);
    sm->enable();
    sm->processEvent(StateEvent::HOME_REQUEST);
    sm->processEvent(StateEvent::HOME_COMPLETE);

    EXPECT_TRUE(sm->setMode(RobotMode::AUTO));
    EXPECT_EQ(sm->currentMode(), RobotMode::AUTO);
}

// E-Stop tests
TEST_F(StateMachineTest, EStopFromAnyState) {
    sm->processEvent(StateEvent::INIT_COMPLETE);
    sm->enable();
    sm->processEvent(StateEvent::HOME_REQUEST);
    sm->processEvent(StateEvent::HOME_COMPLETE);
    sm->processEvent(StateEvent::MOTION_START);

    // E-Stop while moving
    EXPECT_TRUE(sm->processEvent(StateEvent::ESTOP_PRESSED));
    EXPECT_EQ(sm->currentState(), RobotState::ESTOP);
    EXPECT_FALSE(sm->isEnabled());
}

TEST_F(StateMachineTest, ResetFromEStop) {
    sm->processEvent(StateEvent::INIT_COMPLETE);
    sm->processEvent(StateEvent::ESTOP_PRESSED);
    EXPECT_EQ(sm->currentState(), RobotState::ESTOP);

    // Cannot reset while E-Stop active
    EXPECT_FALSE(sm->reset());

    // Release E-Stop first
    sm->processEvent(StateEvent::ESTOP_RELEASED);
    EXPECT_EQ(sm->currentState(), RobotState::ERROR);

    // Now can reset
    EXPECT_TRUE(sm->reset());
    EXPECT_EQ(sm->currentState(), RobotState::IDLE);
}

// Error tests
TEST_F(StateMachineTest, ErrorSetsState) {
    sm->processEvent(StateEvent::INIT_COMPLETE);
    sm->setError(ErrorCode::MOTION_LIMIT_EXCEEDED, "Joint 1 limit exceeded");

    EXPECT_TRUE(sm->hasError());
    EXPECT_EQ(sm->lastError(), ErrorCode::MOTION_LIMIT_EXCEEDED);
}

TEST_F(StateMachineTest, ClearError) {
    sm->processEvent(StateEvent::INIT_COMPLETE);
    sm->setError(ErrorCode::MOTION_LIMIT_EXCEEDED);

    sm->clearError();
    EXPECT_EQ(sm->lastError(), ErrorCode::NONE);
}

// Callback tests
TEST_F(StateMachineTest, StateChangeCallback) {
    bool callbackCalled = false;
    RobotState oldState, newState;

    sm->setStateChangeCallback([&](RobotState o, RobotState n) {
        callbackCalled = true;
        oldState = o;
        newState = n;
    });

    sm->processEvent(StateEvent::INIT_COMPLETE);

    EXPECT_TRUE(callbackCalled);
    EXPECT_EQ(oldState, RobotState::INIT);
    EXPECT_EQ(newState, RobotState::IDLE);
}
