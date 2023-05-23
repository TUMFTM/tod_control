// Copyright 2021 Andreas Schimpe
#include "TrajectoryHandler.h"

void TrajectoryHandler::trajectory_callback(const tod_msgs::TrajectoryConstPtr &msg) {
    _trajectory = *msg;
    _progress.clear();
    _progress.push_back(0.0);
    for (int i=1; i < _trajectory.points.size(); ++i) {
        double dr = tod_helper::Geometry::calc_horizontal_distance(
            _trajectory.points.at(i-1).pose.pose.position, _trajectory.points.at(i).pose.pose.position);
        _progress.push_back(_progress.at(i-1) + dr);
    }
}

tod_msgs::TrajectoryPoint TrajectoryHandler::get_tracking_point(
    const nav_msgs::Odometry &odom, const double lookaheadDistance, double &outTrackingError) const {
    // from current odom, get closest waypoints of desired trajectory
    int idx0{0}, idx1{1};
    closest_two_waypoints_from(odom.pose.pose.position, idx0, idx1);
    // from current progress along desired trajectory, interpolate new target waypoints
    geometry_msgs::Point perpendicular;
    const double inter_waypoint_progress = tod_helper::Geometry::perpendicular_from_pt_on_line(
        odom.pose.pose.position, _trajectory.points.at(idx0).pose.pose.position,
        _trajectory.points.at(idx1).pose.pose.position, perpendicular);
    outTrackingError = tod_helper::Geometry::calc_horizontal_distance(odom.pose.pose.position, perpendicular);
    // add lookahead and interpolate point
    const double currentProgress = _progress.at(idx0) + inter_waypoint_progress;
    const double trackingPtProgress = currentProgress + lookaheadDistance;
    closest_two_progress_from(trackingPtProgress, idx0, idx1, 0);
    return interpolate_pt_for_progress(trackingPtProgress, idx0, idx1);
}

tod_msgs::Trajectory TrajectoryHandler::get_next_waypoints(const nav_msgs::Odometry &odom, const double dt,
                                                           const int nofwps, double &outTrackingError) const {
    tod_msgs::Trajectory waypoints;
    waypoints.header = _trajectory.header;
    waypoints.child_frame_id = _trajectory.child_frame_id;

    // from current odom, get closest waypoints of desired trajectory
    int idx0{0}, idx1{1};
    closest_two_waypoints_from(odom.pose.pose.position, idx0, idx1);
    if (idx1 == _trajectory.points.size()-1) {
        // if reached end of received trajectory, return last waypoint only
        waypoints.points.insert(waypoints.points.end(), nofwps, _trajectory.points.back());
        outTrackingError = tod_helper::Geometry::calc_horizontal_distance(
            odom.pose.pose.position, waypoints.points.back().pose.pose.position);
        return waypoints;
    }

    // from current progress along desired trajectory, interpolate new target waypoints
    geometry_msgs::Point perpendicular;
    const double inter_waypoint_progress = tod_helper::Geometry::perpendicular_from_pt_on_line(
        odom.pose.pose.position, _trajectory.points.at(idx0).pose.pose.position,
        _trajectory.points.at(idx1).pose.pose.position, perpendicular);
    outTrackingError = tod_helper::Geometry::calc_horizontal_distance(odom.pose.pose.position, perpendicular);
    double currentProgress = _progress.at(idx0) + inter_waypoint_progress;
    for (int i = 0; i < nofwps; ++i) {
        double newProgress = currentProgress + dt * _trajectory.points.at(idx0).twist.twist.linear.x;
        closest_two_progress_from(newProgress, idx0, idx1, std::max(idx0-1, 0));
        waypoints.points.emplace_back(interpolate_pt_for_progress(newProgress, idx0, idx1));
        currentProgress = newProgress;
    }
    return waypoints;
}

void TrajectoryHandler::closest_two_waypoints_from(const geometry_msgs::Point &point, int &idx0, int &idx1) const {
    using tod_helper::Geometry::calc_horizontal_distance;
    double minDistance{100000.0};
    int minIdx{0};
    for (int i=0; i < _trajectory.points.size(); ++i) {
        double dist = calc_horizontal_distance(point, _trajectory.points.at(i).pose.pose.position);
        if (dist < minDistance) {
            minDistance = dist;
            minIdx = i;
        }
    }
    if (minIdx == 0) {
        idx0 = minIdx;
        idx1 = minIdx + 1;
    } else if (minIdx == _trajectory.points.size()-1) {
        idx0 = minIdx - 1;
        idx1 = minIdx;
    } else {
        const int prevIdx = std::max(minIdx-1, 0);
        const int nextIdx = std::min(minIdx+1, int(_trajectory.points.size()-1));
        const double distPrev = calc_horizontal_distance(point, _trajectory.points.at(prevIdx).pose.pose.position);
        const double distNext = calc_horizontal_distance(point, _trajectory.points.at(nextIdx).pose.pose.position);
        if (distPrev < distNext) {
            idx0 = prevIdx;
            idx1 = minIdx;
        } else {
            idx0 = minIdx;
            idx1 = nextIdx;
        }
    }
}

void TrajectoryHandler::closest_two_progress_from(const double progress, int &idx0, int &idx1, const int idxOffset) const {
    double minProgressDiff{100000.0};
    int minIdx{idxOffset};
    for (int i=idxOffset; i < _trajectory.points.size(); ++i) {
        double progressDiff = std::abs(_progress.at(i) - progress);
        if (progressDiff < minProgressDiff) {
            minProgressDiff = progressDiff;
            minIdx = i;
        }
    }
    const int prevIdx = std::max(minIdx-1, idxOffset);
    const int nextIdx = std::min(minIdx+1, int(_trajectory.points.size()-1));
    const double progressDiffPrev = std::abs(_progress.at(prevIdx) - progress);
    const double progressDiffNext = std::abs(_progress.at(nextIdx) - progress);
    if (progressDiffPrev < progressDiffNext) {
        idx0 = prevIdx;
        idx1 = minIdx;
    } else {
        idx0 = minIdx;
        idx1 = nextIdx;
    }
}

tod_msgs::TrajectoryPoint TrajectoryHandler::interpolate_pt_for_progress(const double targetProgress, const int idx0, const int idx1) const {
    if (idx0 == idx1 && idx0 == _progress.size()-1) {
        return _trajectory.points.at(idx0);
    }
    const double progressToInterpolate = targetProgress - _progress.at(idx0);
    const geometry_msgs::Point& pt0 = _trajectory.points.at(idx0).pose.pose.position;
    const geometry_msgs::Point& pt1 = _trajectory.points.at(idx1).pose.pose.position;
    const double dx = pt1.x - pt0.x;
    const double dy = pt1.y - pt0.y;
    const double norm = std::sqrt(dx*dx + dy*dy);
    tod_msgs::TrajectoryPoint tpt = _trajectory.points.at(idx0);
    tpt.pose.pose.position.x = pt0.x + progressToInterpolate * dx / norm;
    tpt.pose.pose.position.y = pt0.y + progressToInterpolate * dy / norm;
    return tpt;
}
