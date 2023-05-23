// Copyright 2021 Andreas Schimpe
#pragma once
#include <iostream>
#include <algorithm>
#include "BicycleModelVariables.h"

namespace tod_shared_control {

class FeedbackLinearization {
public:
    FeedbackLinearization();
    double compute_steering_ctrl_cmd(const BicycleState &currentState, const BicycleState &desiredState,
                                     const double wheelBase, const double dt, double &outLateralError, double &outHeadingError);

private:
    double _gamma1{0.5}, _gamma2{0.7};
    double _maxSteeringRate{60.0 * 3.1415 / 180.0};
    double _previousSteeringCmd{0.0};
};

} // namespace tod_shared_control
