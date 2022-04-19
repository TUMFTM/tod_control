// Copyright 2020 Simon Hoffmann
#include "ConstantDeceleration.h"

double ConstantDeceleration::get_braking_distance(const double velocity, const double offset,
    const double deceleration) {
        return offset + pow(velocity, 2)/(2.0*deceleration);
}

void ConstantDeceleration::apply_velocity_profile(tod_msgs::Trajectory& traj, const double velocity,
        const double offset, const double deceleration) {
    std::vector<double> accumulatedDistance = tod_helper::Trajectory::accumulated_distance(traj);

    ros::Time refStamp = get_stamp_of_ref_point();
    traj.header.stamp = refStamp;
    for (int elem = 0; elem < traj.points.size(); elem++) {
        traj.points.at(elem).twist.twist.linear.x = calc_velocity_with_offset(
            velocity, accumulatedDistance.at(elem), offset, deceleration);
        ros::Time stamp = refStamp + ros::Duration(calc_time_with_offset(velocity,
            accumulatedDistance.at(elem), offset, deceleration));
        traj.points.at(elem).twist.header.stamp = stamp;
        traj.points.at(elem).pose.header.stamp = stamp;
    }
}

double constexpr ConstantDeceleration::calc_velocity_with_offset(double v0, double s, double s0, double a) {
    return (s <= s0) ? v0 : calc_velocity(s-s0, v0, a);
}

double constexpr ConstantDeceleration::calc_velocity(const double s, const double v0, const double a) {
    double square_vel = pow(v0, 2) - 2 * a * s;
    return  (square_vel > 0) ? sqrt(square_vel) : 0.0;
}

double ConstantDeceleration::calc_time_with_offset(double v0, double s, double s0, double a) {
    if (v0 == 0)
        return 0;
    return (s <= s0) ? s / v0 : s0/v0 + calc_time(v0, s-s0, a);
}

double ConstantDeceleration::calc_time(const double v0, const double s, const double a) {
    if (a == 0) {
        ROS_WARN_STREAM_ONCE(ros::this_node::getName() << ": Deceleration set to Zero!");
        return s / v0;
    }
    double square_vel = pow(v0, 2) - 2 * a * s;
    return (square_vel > 0) ? ((- v0 + sqrt(square_vel)) / -a) : v0/a;
}
