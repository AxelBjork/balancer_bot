// This test is disabled because it tests Stepper::stepN which no longer exists.
// The MotorRunner now uses DualWave for pulse generation.
// TODO: Rewrite tests for DualWave/MotorRunner.

#include <gmock/gmock.h>
#include <gtest/gtest.h>

TEST(MotorRunnerTest, Disabled) { SUCCEED(); }
