// Copyright 2020 Feiler

#include <gtest/gtest.h>
#include "PercModStatus.h"
#include <cmath>

class test_PercModStatus : public ::testing::Test {
    protected:
        PercModStatus percModStatus;

};

TEST_F(test_PercModStatus, initializationStatusCheck) {
    EXPECT_EQ(percModStatus.getState(), (int) PerceptionModificationMode::OPERATOR_PLANNING);
}

TEST_F(test_PercModStatus, increaseOnce) {
    percModStatus.increaseState();
    EXPECT_EQ(percModStatus.getState(),
        (int) PerceptionModificationMode::OPERATOR_SENT_MODIFICATION_REQUEST);
}

TEST_F(test_PercModStatus, increaseTwice) {
    percModStatus.increaseState();
    percModStatus.increaseState();
    EXPECT_EQ(percModStatus.getState(),
        (int) PerceptionModificationMode::OPERATOR_APPROVING);
}

TEST_F(test_PercModStatus, increaseToOften) {
    percModStatus.increaseState();
    percModStatus.increaseState();
    percModStatus.increaseState();
    percModStatus.increaseState();
    percModStatus.increaseState();
    EXPECT_EQ(percModStatus.getState(),
        (int) PerceptionModificationMode::OPERATOR_DRIVING);
}

TEST_F(test_PercModStatus, resetState) {
    // increase to some state
    percModStatus.increaseState();
    percModStatus.resetState();
    EXPECT_EQ(percModStatus.getState(), (int) PerceptionModificationMode::OPERATOR_PLANNING);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}