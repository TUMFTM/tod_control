// Copyright 2021 Andreas Schimpe
#include "FeedbackLinearization.h"

namespace tod_shared_control {

FeedbackLinearization::FeedbackLinearization() {

}

double FeedbackLinearization::compute_steering_ctrl_cmd(const BicycleState &currentState, const BicycleState &desiredState,
                                                        const double wheelBase, const double dt,
                                                        double &outLateralError, double &outHeadingError) {
    double lateralError = -sin(desiredState.Heading) * (currentState.PosX - desiredState.PosX) +
                          cos(desiredState.Heading) * (currentState.PosY - desiredState.PosY);
    double headingError = currentState.Heading - desiredState.Heading;
    headingError = std::atan(std::sin(headingError) / std::cos(headingError));
    if (currentState.Cmd.Velocity < 3.75) {
        double zeroSteeringCmd = 0.0;
        return zeroSteeringCmd;
    }
    const double denominator = pow(currentState.Cmd.Velocity, 2) * cos(headingError);
    const double numerator = wheelBase * (-_gamma1 * lateralError - _gamma2 * currentState.Cmd.Velocity * sin(headingError));
    double steeringCmd = std::atan2(numerator, denominator);
    // clamp according to steering rate
    const double deltaSteeringMax = _maxSteeringRate * dt;
    steeringCmd = std::clamp(steeringCmd, _previousSteeringCmd - deltaSteeringMax, _previousSteeringCmd + deltaSteeringMax);
    _previousSteeringCmd = steeringCmd;
    outLateralError = lateralError;
    outHeadingError = headingError;
    return steeringCmd;
}

} // namespace tod_shared_control
