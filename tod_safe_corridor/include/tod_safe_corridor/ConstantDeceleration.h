// Copyright 2020 Simon Hoffmann
#pragma once
#include <ros/ros.h>
#include "tod_msgs/Trajectory.h"
#include "tod_helper/geometry/Helpers.h"
#include "tod_helper/trajectory/Helpers.h"
#include <algorithm>
#include <math.h>
#include "LongitudinalPlanner.h"

class ConstantDeceleration : public LongitudinalPlanner {
public:
    ConstantDeceleration() = default;
    void apply_velocity_profile(tod_msgs::Trajectory& traj, const double velocity, const double offset,
        const double deceleration) override;
    double get_braking_distance(const double velocity, const double offset, const double deceleration) override;
private:
    static constexpr double calc_velocity(const double s, const double v0, const double a);
    static double calc_time(const double v0, const double s, const double a);
    static constexpr double calc_velocity_with_offset(const double v0, const double s, const double s0, double a);
    static double calc_time_with_offset(double v0, double s, double s0, double a);
};
