// Copyright 2021 Andreas Schimpe
#pragma once
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tod_msgs/Trajectory.h>
#include <tod_helper/geometry/Helpers.h>

class TrajectoryHandler {
public:
    TrajectoryHandler() {}
    ~TrajectoryHandler() {}
    void trajectory_callback(const tod_msgs::TrajectoryConstPtr &msg);
    tod_msgs::TrajectoryPoint get_tracking_point(
        const nav_msgs::Odometry &odom, const double lookaheadDistance, double &outTrackingError) const;
    tod_msgs::Trajectory get_next_waypoints(
        const nav_msgs::Odometry &odom, const double dt, const int nofwps, double &outTrackingError) const;
    std::string get_frame_id() const { return _trajectory.header.frame_id; }
    bool has_trajectory() const { return !_trajectory.points.empty(); }

private:
    tod_msgs::Trajectory _trajectory;
    std::vector<double> _progress;
    void closest_two_waypoints_from(const geometry_msgs::Point &point, int &idx0, int &idx1) const;
    void closest_two_progress_from(const double progress, int &idx0, int &idx1, const int idxOffset) const;
    tod_msgs::TrajectoryPoint interpolate_pt_for_progress(const double targetProgress, const int idx0, const int idx1) const;
};
