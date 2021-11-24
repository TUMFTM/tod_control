// Copyright 2020 Simon Hoffmann
#include "tod_pure_pursuit/PurePursuit.h"

PurePursuit::PurePursuit(ros::NodeHandle& nodeHandle) {
    _vehParam = tod_helper::Vehicle::Parameters(nodeHandle);
    _ppConfig = PurePursuitConfig(nodeHandle);
}

bool PurePursuit::calc_control_command(const tod_msgs::TrajectoryConstPtr trajectory,
        const geometry_msgs::PoseStamped& poseRearAxle, tod_msgs::PrimaryControlCmd& cmd,
        const double currentVelocity) {
    _ppConfig.set_lookahead_distance(currentVelocity);
    int _nextWaypoint = find_next_point(trajectory, poseRearAxle);

    if (_nextWaypoint == -1) {
        ROS_DEBUG_STREAM("Lost Trajectory");
        cmd.velocity = cmd.steeringWheelAngle = 0.0;
        return false;
    }
    double kappa =  calc_curvature(trajectory->points.at(_nextWaypoint).pose.pose.position, poseRearAxle);
    cmd.velocity = _desiredSpeed;
    cmd.steeringWheelAngle = tod_helper::Vehicle::Model::rwa2swa(
        tod_helper::Vehicle::Model::curvature_to_rwa(_vehParam.get_wheel_base(), kappa),
        _vehParam.get_max_swa_rad(), _vehParam.get_max_rwa_rad());

    return true;
}

double PurePursuit::calc_curvature(const geometry_msgs::Point &target, const geometry_msgs::PoseStamped& poseRearAxle) {
    static double distToTarget, relPosToVehicleY;
    distToTarget = tod_helper::Geometry::calc_horizontal_distance(target, poseRearAxle.pose.position);
    relPosToVehicleY = tod_helper::Geometry::calc_relative_position(target, poseRearAxle.pose).y;

    return distToTarget != 0 ? (2 * relPosToVehicleY) / pow(distToTarget, 2) : 0.0;
}

int PurePursuit::find_next_point(const tod_msgs::TrajectoryConstPtr _trajectory,
        const geometry_msgs::PoseStamped& _poseRearAxle) {
    int closest_waypoint = tod_helper::Trajectory::get_closest_trajectory_point_in_x_dir(_trajectory,
        _poseRearAxle.pose);

    if (closest_waypoint == -1)
        return -1;

    _desiredSpeed = _trajectory->points.at(closest_waypoint).twist.twist.linear.x;

    auto lookAheadPoint = std::find_if(_trajectory->points.begin()+closest_waypoint, _trajectory->points.end(),
        [&_poseRearAxle, this](const auto& point) {
            return tod_helper::Geometry::calc_horizontal_distance(point.pose.pose.position,
                _poseRearAxle.pose.position) > _ppConfig.lookaheadDistance;
        });

    if (lookAheadPoint == _trajectory->points.end())
        return -1;

    return std::distance(_trajectory->points.begin(), lookAheadPoint);
}

std::string PurePursuit::get_rearAxleFrameId() {
   return _ppConfig.rearAxleFrameId;
}
