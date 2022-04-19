// Copyright 2020 Simon Hoffmann
#include "ClothoidPlanner.h"

ClothoidPlanner::ClothoidPlanner(std::shared_ptr<tod_core::VehicleParameters> vehParams, float segmentLength) :
        LateralPlanner(vehParams, segmentLength) {
}

std::vector<geometry_msgs::Point> ClothoidPlanner::numeric_integration_position(const double c_0, const double c_1,
        const double length, const int n_increments) {
    std::vector<geometry_msgs::Point> integrationPoses;
    float ds = length/(n_increments-1);
    integrationPoses.clear();
    integrationPoses.resize(n_increments);
    integrationPoses.at(0) = _offsetPosition;

    // Numeric integration of Fresnel Integrals of Clothoid
    for (int incr =0; incr < n_increments-1; incr++) {
        float s = (ds/2 + ds*incr);
        integrationPoses.at(incr+1).y = integrationPoses.at(incr).y
            + std::sin(calc_orientation(c_0, c_1, s, _offsetOrientation))*ds;
        integrationPoses.at(incr+1).x = integrationPoses.at(incr).x
            + std::cos(calc_orientation(c_0, c_1, s, _offsetOrientation))*ds;
    }
    return integrationPoses;
}

constexpr double ClothoidPlanner::calc_orientation(const double c_0, const double c_1, const double s,
        const double yaw_0) noexcept {
    return (c_0*s + 0.5*c_1*s*s) + yaw_0;
}

void ClothoidPlanner::update_pose(const double x, const double y, const double yaw) {
    _offsetPosition.x = x;
    _offsetPosition.y = y;
    _offsetOrientation = yaw;
}

tod_msgs::Trajectory ClothoidPlanner::calculate_path(const tod_msgs::PrimaryControlCmd& cmd,
        const std::string& frame_id, const std::string& child_frame_id, const double length) {
    tod_msgs::Trajectory trajectory;
    double c0 = tod_helper::Vehicle::Model::rwa_to_curvature(_vehParams->get_wheel_base(),
        tod_helper::Vehicle::Model::swa2rwa(cmd.steeringWheelAngle, _vehParams->get_max_swa_rad(),
        _vehParams->get_max_rwa_rad()));
    double c1 = calc_curvature_rate(c0, cmd.header.stamp.toNSec(), cmd.velocity);

    int poseElements = ceil(length/_segmentLength);
    int integrationIncrements = poseElements; //use same ammout for better results

    tod_msgs::TrajectoryPoint point;
    point.pose.header.frame_id = frame_id;
    point.twist.header.frame_id = child_frame_id;

    trajectory.header.frame_id = frame_id;
    trajectory.child_frame_id = child_frame_id;

    auto integrationPoses = numeric_integration_position(c0, c1, length, integrationIncrements);
    // Calculate n Poses
    for (int elem = 0; elem < poseElements; elem++) {
        double s = get_arc_length_from_pose(elem, poseElements, length);
        int increment = ceil(((double)integrationIncrements/(double)poseElements)* (double)elem);

        //POSITION
        point.pose.pose.position = integrationPoses.at(increment);

        // ORIENTATION
        tf2::Quaternion quat;
        quat.setRPY(0, 0, calc_orientation(c0, c1, s, _offsetOrientation));
        point.pose.pose.orientation = tf2::toMsg(quat);

        //PUSH_BACK
        trajectory.points.push_back(point);
    }
    return trajectory;
}

double ClothoidPlanner::calc_curvature_rate(const float Curvature, const u_int64_t timeStampNs, const double velocity) {
    static double prevCurvature{ 0 };
    static u_int64_t prevStamp{ 0 };
    static double c1{ 0 };

    double dt = (timeStampNs - prevStamp);
    if (dt >= 80000000) {
        //assuming constant velocity between 2 measuring points
        c1 = (velocity < 1e-5) ? 0.0 : (Curvature - prevCurvature) / (dt * velocity);
        prevCurvature = Curvature;
        prevStamp = timeStampNs;
    }
    return c1;
}

constexpr double ClothoidPlanner::get_arc_length_from_pose(const int trajectoryElement, const int total_elements,
        const double length) noexcept {
    return (length/(total_elements-1)) * trajectoryElement;
}
