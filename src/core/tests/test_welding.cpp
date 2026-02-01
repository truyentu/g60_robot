#include <gtest/gtest.h>
#include "welding/WeldingTypes.hpp"
#include "welding/WeldingStateMachine.hpp"
#include "welding/WeldingController.hpp"
#include "welding/WeldingIO.hpp"

using namespace robotics::welding;

// ============================================================================
// Welding Types Tests
// ============================================================================

TEST(WeldingTypesTest, DefaultTimings) {
    WeldingTimings timings;

    EXPECT_EQ(timings.preFlowTime, 300u);
    EXPECT_EQ(timings.postFlowTime, 500u);
    EXPECT_EQ(timings.arcRetryCount, 3u);
}

TEST(WeldingTypesTest, DefaultParams) {
    WeldingParams params;

    EXPECT_GT(params.current, 0);
    EXPECT_GT(params.voltage, 0);
    EXPECT_GT(params.wireSpeed, 0);
}

TEST(WeldingTypesTest, DefaultJob) {
    WeldingJob job;

    EXPECT_EQ(job.process, WeldingProcess::MIG_MAG);
    EXPECT_EQ(job.transferMode, TransferMode::SHORT_ARC);
    EXPECT_TRUE(job.synergicMode);
}

TEST(WeldingTypesTest, DefaultFeedback) {
    WeldingFeedback feedback;

    EXPECT_EQ(feedback.actualCurrent, 0);
    EXPECT_EQ(feedback.actualVoltage, 0);
    EXPECT_FALSE(feedback.arcPresent);
}

TEST(WeldingTypesTest, DefaultStatus) {
    WeldingStatus status;

    EXPECT_EQ(status.state, WeldingState::IDLE);
    EXPECT_EQ(status.fault, WeldingFault::NONE);
    EXPECT_FALSE(status.isWelding);
    EXPECT_FALSE(status.hasFault);
}

// ============================================================================
// State Machine Tests
// ============================================================================

class WeldingStateMachineTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Set ready inputs
        inputs_.powerSourceReady = true;
        inputs_.gasFlowOk = true;
        inputs_.wireOk = true;

        sm_.updateInputs(inputs_);
    }

    WeldingStateMachine sm_;
    WeldingInputs inputs_;
};

TEST_F(WeldingStateMachineTest, InitialStateIsIdle) {
    EXPECT_EQ(sm_.getState(), WeldingState::IDLE);
}

TEST_F(WeldingStateMachineTest, StartWeld_TransitionsToPreFlow) {
    sm_.processEvent(WeldingEvent::START_WELD);

    EXPECT_EQ(sm_.getState(), WeldingState::PRE_FLOW);
}

TEST_F(WeldingStateMachineTest, PreFlow_GasValveOn) {
    sm_.processEvent(WeldingEvent::START_WELD);

    auto outputs = sm_.getOutputs();
    EXPECT_TRUE(outputs.gasValve);
}

TEST_F(WeldingStateMachineTest, PreFlow_TimerTransitionsToArcStart) {
    WeldingJob job;
    job.timings.preFlowTime = 100;
    sm_.setJob(job);

    sm_.processEvent(WeldingEvent::START_WELD);
    EXPECT_EQ(sm_.getState(), WeldingState::PRE_FLOW);

    // Simulate time passing
    sm_.update(50);
    EXPECT_EQ(sm_.getState(), WeldingState::PRE_FLOW);

    sm_.update(60);  // Total 110ms > 100ms
    EXPECT_EQ(sm_.getState(), WeldingState::ARC_START);
}

TEST_F(WeldingStateMachineTest, ArcStart_WireFeedAndArcEnabled) {
    WeldingJob job;
    job.timings.preFlowTime = 0;
    sm_.setJob(job);

    sm_.processEvent(WeldingEvent::START_WELD);
    sm_.update(1);

    auto outputs = sm_.getOutputs();
    EXPECT_TRUE(outputs.gasValve);
    EXPECT_TRUE(outputs.wireFeed);
    EXPECT_TRUE(outputs.arcEnable);
    EXPECT_TRUE(outputs.torchTrigger);
}

TEST_F(WeldingStateMachineTest, ArcDetected_TransitionsToWelding) {
    WeldingJob job;
    job.timings.preFlowTime = 0;
    sm_.setJob(job);

    sm_.processEvent(WeldingEvent::START_WELD);
    sm_.update(1);

    // Simulate arc established
    inputs_.arcDetect = true;
    sm_.updateInputs(inputs_);

    WeldingFeedback feedback;
    feedback.arcPresent = true;
    feedback.actualCurrent = 180;
    feedback.actualVoltage = 22;
    sm_.updateFeedback(feedback);

    sm_.update(1);

    EXPECT_EQ(sm_.getState(), WeldingState::WELDING);
}

TEST_F(WeldingStateMachineTest, StopWeld_TransitionsToCraterFill) {
    WeldingJob job;
    job.timings.preFlowTime = 0;
    sm_.setJob(job);

    sm_.processEvent(WeldingEvent::START_WELD);
    sm_.update(1);

    // Go to welding state
    inputs_.arcDetect = true;
    sm_.updateInputs(inputs_);
    sm_.update(1);

    EXPECT_EQ(sm_.getState(), WeldingState::WELDING);

    // Stop weld
    sm_.processEvent(WeldingEvent::STOP_WELD);
    EXPECT_EQ(sm_.getState(), WeldingState::CRATER_FILL);
}

TEST_F(WeldingStateMachineTest, FullWeldSequence) {
    WeldingJob job;
    job.timings.preFlowTime = 10;
    job.timings.craterFillTime = 10;
    job.timings.burnBackTime = 10;
    job.timings.postFlowTime = 10;
    sm_.setJob(job);

    // Start
    sm_.processEvent(WeldingEvent::START_WELD);
    EXPECT_EQ(sm_.getState(), WeldingState::PRE_FLOW);

    // Pre-flow complete
    sm_.update(20);
    EXPECT_EQ(sm_.getState(), WeldingState::ARC_START);

    // Arc established
    inputs_.arcDetect = true;
    sm_.updateInputs(inputs_);
    sm_.update(1);
    EXPECT_EQ(sm_.getState(), WeldingState::WELDING);

    // Stop weld
    sm_.processEvent(WeldingEvent::STOP_WELD);
    EXPECT_EQ(sm_.getState(), WeldingState::CRATER_FILL);

    // Crater fill complete
    sm_.update(20);
    EXPECT_EQ(sm_.getState(), WeldingState::ARC_END);

    // Arc end -> burn back
    sm_.update(250);
    EXPECT_EQ(sm_.getState(), WeldingState::BURN_BACK);

    // Burn back complete
    sm_.update(20);
    EXPECT_EQ(sm_.getState(), WeldingState::POST_FLOW);

    // Post flow complete
    sm_.update(20);
    EXPECT_EQ(sm_.getState(), WeldingState::IDLE);
}

TEST_F(WeldingStateMachineTest, EmergencyStop_FromAnyState) {
    sm_.processEvent(WeldingEvent::START_WELD);
    sm_.update(1);

    sm_.processEvent(WeldingEvent::EMERGENCY_STOP);

    EXPECT_EQ(sm_.getState(), WeldingState::EMERGENCY_STOP);

    // All outputs should be off
    auto outputs = sm_.getOutputs();
    EXPECT_FALSE(outputs.gasValve);
    EXPECT_FALSE(outputs.wireFeed);
    EXPECT_FALSE(outputs.arcEnable);
}

TEST_F(WeldingStateMachineTest, Abort_StopsImmediately) {
    sm_.processEvent(WeldingEvent::START_WELD);
    sm_.update(100);

    sm_.processEvent(WeldingEvent::ABORT);

    EXPECT_EQ(sm_.getState(), WeldingState::IDLE);
}

TEST_F(WeldingStateMachineTest, ArcFail_AfterRetries) {
    WeldingJob job;
    job.timings.preFlowTime = 0;
    job.timings.arcStartTime = 50;
    job.timings.arcRetryCount = 2;
    sm_.setJob(job);

    sm_.processEvent(WeldingEvent::START_WELD);
    sm_.update(1);

    // Arc never establishes
    sm_.update(60);  // First attempt
    sm_.update(60);  // Second attempt
    sm_.update(60);  // Third attempt (exceeds retries)

    EXPECT_EQ(sm_.getState(), WeldingState::FAULT);
    EXPECT_EQ(sm_.getFault(), WeldingFault::ARC_FAIL);
}

TEST_F(WeldingStateMachineTest, Reset_ClearsFault) {
    WeldingJob job;
    job.timings.preFlowTime = 0;
    job.timings.arcStartTime = 10;
    job.timings.arcRetryCount = 0;
    sm_.setJob(job);

    // Trigger arc fail
    sm_.processEvent(WeldingEvent::START_WELD);
    sm_.update(1);
    sm_.update(20);  // Arc timeout

    EXPECT_EQ(sm_.getState(), WeldingState::FAULT);

    sm_.processEvent(WeldingEvent::RESET);

    EXPECT_EQ(sm_.getState(), WeldingState::IDLE);
    EXPECT_FALSE(sm_.hasFault());
}

TEST_F(WeldingStateMachineTest, SafetyFault_WhenNotReady) {
    // Set power source not ready
    inputs_.powerSourceReady = false;
    sm_.updateInputs(inputs_);

    sm_.processEvent(WeldingEvent::START_WELD);

    EXPECT_EQ(sm_.getState(), WeldingState::FAULT);
    EXPECT_EQ(sm_.getFault(), WeldingFault::SAFETY_FAULT);
}

TEST_F(WeldingStateMachineTest, IsReady_WhenIdleAndNoFault) {
    EXPECT_TRUE(sm_.isReady());

    inputs_.powerSourceReady = false;
    sm_.updateInputs(inputs_);

    EXPECT_FALSE(sm_.isReady());
}

TEST_F(WeldingStateMachineTest, IsWelding_WhenInWeldingOrCraterFill) {
    EXPECT_FALSE(sm_.isWelding());

    WeldingJob job;
    job.timings.preFlowTime = 0;
    sm_.setJob(job);

    sm_.processEvent(WeldingEvent::START_WELD);
    sm_.update(1);

    inputs_.arcDetect = true;
    sm_.updateInputs(inputs_);
    sm_.update(1);

    EXPECT_TRUE(sm_.isWelding());
}

TEST_F(WeldingStateMachineTest, StateChangeCallback) {
    WeldingState oldState = WeldingState::IDLE;
    WeldingState newState = WeldingState::IDLE;
    int callCount = 0;

    sm_.setStateChangeCallback([&](WeldingState o, WeldingState n) {
        oldState = o;
        newState = n;
        callCount++;
    });

    sm_.processEvent(WeldingEvent::START_WELD);

    EXPECT_EQ(oldState, WeldingState::IDLE);
    EXPECT_EQ(newState, WeldingState::PRE_FLOW);
    EXPECT_GE(callCount, 1);
}

// ============================================================================
// Welding Controller Tests
// ============================================================================

TEST(WeldingControllerTest, Initialize) {
    WeldingController controller;

    bool result = controller.initialize();

    EXPECT_TRUE(result);
}

TEST(WeldingControllerTest, StartWeld) {
    WeldingController controller;
    controller.initialize();

    // Set up ready inputs
    WeldingInputs inputs;
    inputs.powerSourceReady = true;
    inputs.gasFlowOk = true;
    controller.updateInputs(inputs);

    WeldingJob job;
    bool result = controller.startWeld(job);

    EXPECT_TRUE(result);
    EXPECT_NE(controller.getState(), WeldingState::IDLE);
}

TEST(WeldingControllerTest, StopWeld) {
    WeldingController controller;
    controller.initialize();

    WeldingInputs inputs;
    inputs.powerSourceReady = true;
    inputs.gasFlowOk = true;
    controller.updateInputs(inputs);

    WeldingJob job;
    job.timings.preFlowTime = 0;
    controller.startWeld(job);

    // Let it progress
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    controller.stopWeld();

    // Should be stopping or stopped
    auto state = controller.getState();
    EXPECT_TRUE(state == WeldingState::CRATER_FILL ||
                state == WeldingState::ARC_END ||
                state == WeldingState::BURN_BACK ||
                state == WeldingState::POST_FLOW ||
                state == WeldingState::IDLE ||
                state == WeldingState::ARC_START);
}

TEST(WeldingControllerTest, EmergencyStop) {
    WeldingController controller;
    controller.initialize();

    WeldingInputs inputs;
    inputs.powerSourceReady = true;
    inputs.gasFlowOk = true;
    controller.updateInputs(inputs);

    WeldingJob job;
    controller.startWeld(job);

    controller.emergencyStop();

    EXPECT_EQ(controller.getState(), WeldingState::EMERGENCY_STOP);
}

TEST(WeldingControllerTest, Reset) {
    WeldingController controller;
    controller.initialize();

    WeldingInputs inputs;
    inputs.powerSourceReady = true;
    inputs.gasFlowOk = true;
    controller.updateInputs(inputs);

    WeldingJob job;
    controller.startWeld(job);
    controller.emergencyStop();

    EXPECT_EQ(controller.getState(), WeldingState::EMERGENCY_STOP);

    controller.reset();

    EXPECT_EQ(controller.getState(), WeldingState::IDLE);
}

TEST(WeldingControllerTest, AdjustParameters) {
    WeldingController controller;
    controller.initialize();

    controller.adjustCurrent(200.0f);
    controller.adjustVoltage(25.0f);
    controller.adjustWireSpeed(10.0f);

    EXPECT_FLOAT_EQ(controller.getCurrentJob().params.current, 200.0f);
    EXPECT_FLOAT_EQ(controller.getCurrentJob().params.voltage, 25.0f);
    EXPECT_FLOAT_EQ(controller.getCurrentJob().params.wireSpeed, 10.0f);
}

// ============================================================================
// Simulated I/O Tests
// ============================================================================

TEST(SimulatedIOTest, DefaultReady) {
    SimulatedWeldingIO io;

    auto inputs = io.readInputs();

    EXPECT_TRUE(inputs.powerSourceReady);
    EXPECT_TRUE(inputs.gasFlowOk);
    EXPECT_TRUE(inputs.wireOk);
}

TEST(SimulatedIOTest, SetOutputs) {
    SimulatedWeldingIO io;

    WeldingOutputs outputs;
    outputs.gasValve = true;
    outputs.arcEnable = true;

    io.setAllOutputs(outputs);

    auto readOutputs = io.getOutputs();
    EXPECT_TRUE(readOutputs.gasValve);
    EXPECT_TRUE(readOutputs.arcEnable);
}

TEST(SimulatedIOTest, ArcSimulation) {
    SimulatedWeldingIO io;

    io.setCurrentCommand(180);
    io.setVoltageCommand(22);

    // Before arc enabled
    EXPECT_EQ(io.readCurrent(), 0);

    // Enable arc
    WeldingOutputs outputs;
    outputs.arcEnable = true;
    outputs.torchTrigger = true;
    io.setAllOutputs(outputs);

    // Should read simulated current
    EXPECT_GT(io.readCurrent(), 0);
}

TEST(SimulatedIOTest, WireSpeedSimulation) {
    SimulatedWeldingIO io;

    io.setWireSpeedCommand(8.0f);

    // Before wire feed enabled
    EXPECT_EQ(io.readWireSpeed(), 0);

    WeldingOutputs outputs;
    outputs.wireFeed = true;
    io.setAllOutputs(outputs);

    EXPECT_FLOAT_EQ(io.readWireSpeed(), 8.0f);
}

TEST(SimulatedIOTest, ArcDetection) {
    SimulatedWeldingIO io;

    // Initially no arc
    auto inputs = io.readInputs();
    EXPECT_FALSE(inputs.arcDetect);

    // Enable all outputs for arc
    WeldingOutputs outputs;
    outputs.arcEnable = true;
    outputs.wireFeed = true;
    outputs.torchTrigger = true;
    io.setAllOutputs(outputs);

    inputs = io.readInputs();
    EXPECT_TRUE(inputs.arcDetect);
    EXPECT_TRUE(inputs.currentFlowing);
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
