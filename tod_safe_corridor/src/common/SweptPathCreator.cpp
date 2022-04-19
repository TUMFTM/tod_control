// Copyright 2020 Simon Hoffmann
#include "SweptPathCreator.h"

SweptPathCreator::SweptPathCreator(std::shared_ptr<tod_core::VehicleParameters> vehParams, double bufferFront,
        double bufferSideStat, double longitudinalOffset) {
    _corrParams = std::make_shared<SweptPathParams>(bufferFront, bufferSideStat, longitudinalOffset);
    _vehParams = vehParams;
    init_vehicle_boundaries();
}
SweptPathCreator::SweptPathCreator(std::shared_ptr<tod_core::VehicleParameters> vehParams,
        std::shared_ptr<SweptPathParams> sweptPathParams) {
    _corrParams = sweptPathParams;
    _vehParams = vehParams;
    init_vehicle_boundaries();
}

void SweptPathCreator::apply_dynamic_side_buffer(const double bufferSideDyn) {
    double newWidth = _vehParams->get_width() + _corrParams->bufferSideStat*2.0 + bufferSideDyn*2.0;
    _vehBoundaries.frontLeft.y      = newWidth/2;
    _vehBoundaries.rearLeft.y       = newWidth/2;
    _vehBoundaries.rearAxleLeft.y   = newWidth/2;
    _vehBoundaries.frontRight.y     = -newWidth/2;
    _vehBoundaries.rearRight.y      = -newWidth/2;
    _vehBoundaries.rearAxleRight.y  = -newWidth/2;
}

void SweptPathCreator::init_vehicle_boundaries() {
    apply_dynamic_side_buffer(0.0);

    _vehBoundaries.frontLeft.x      = _vehParams->get_distance_rear_axle()
        + _vehParams->get_distance_front_bumper() + _corrParams->bufferFront;
    _vehBoundaries.frontRight.x     = _vehParams->get_distance_rear_axle()
        + _vehParams->get_distance_front_bumper() + _corrParams->bufferFront;
    _vehBoundaries.rearLeft.x       = _vehParams->get_distance_rear_axle() - _vehParams->get_distance_rear_bumper();
    _vehBoundaries.rearRight.x      = _vehParams->get_distance_rear_axle() - _vehParams->get_distance_rear_bumper();
    _vehBoundaries.rearAxleLeft.x   = 0.0;
    _vehBoundaries.rearAxleRight.x  = 0.0;
}

void SweptPathCreator::create_corridor(geometry_msgs::PolygonStamped& corridor,
        const tod_msgs::Trajectory& trajectory, const double curvature) {
    transform_trajectory_to_vehicle_boundaries(trajectory);
    corridor.polygon.points.clear(); // ensure there is no old content
    corridor.header = trajectory.header;
    if (curvature > 1e-8)
        on_curve_left(corridor);
    else
        on_curve_right(corridor);
}

void SweptPathCreator::create_corridor(geometry_msgs::PolygonStamped& corridor,
        const tod_msgs::Trajectory& trajectory) {
    transform_trajectory_to_vehicle_boundaries(trajectory);
    corridor.polygon.points.clear(); // ensure there is no old content
    corridor.header = trajectory.header;
    if (tod_helper::Geometry::calc_relative_position(
            trajectory.points.back().pose.pose.position, trajectory.points.at(0).pose.pose).y > 1e-7)
        on_curve_left(corridor);
    else
        on_curve_right(corridor);
}

void SweptPathCreator::on_curve_left(geometry_msgs::PolygonStamped& corridor) {
    corridor.polygon.points.push_back(_corrBoundaries.rearLeft);// adding polygon points clockwise
    corridor.polygon.points.insert(corridor.polygon.points.end(), _corrBoundaries.rearAxleLeft.begin(),
        _corrBoundaries.rearAxleLeft.end());
    corridor.polygon.points.push_back(_corrBoundaries.frontLeft.back());
    std::reverse(_corrBoundaries.frontRight.begin(), _corrBoundaries.frontRight.end());
    corridor.polygon.points.insert(corridor.polygon.points.end(), _corrBoundaries.frontRight.begin(),
        _corrBoundaries.frontRight.end());
    corridor.polygon.points.push_back(_corrBoundaries.rearAxleRight.front());
    corridor.polygon.points.push_back(_corrBoundaries.rearRight);
}

void SweptPathCreator::on_curve_right(geometry_msgs::PolygonStamped& corridor) {
    corridor.polygon.points.push_back(_corrBoundaries.rearLeft);// adding polygon points clockwise
    corridor.polygon.points.push_back(_corrBoundaries.rearAxleLeft.front());
    corridor.polygon.points.insert(corridor.polygon.points.end(), _corrBoundaries.frontLeft.begin(),
        _corrBoundaries.frontLeft.end());
    corridor.polygon.points.push_back(_corrBoundaries.frontRight.back());
    std::reverse(_corrBoundaries.rearAxleRight.begin(), _corrBoundaries.rearAxleRight.end());
    corridor.polygon.points.insert(corridor.polygon.points.end(), _corrBoundaries.rearAxleRight.begin(),
        _corrBoundaries.rearAxleRight.end());
    corridor.polygon.points.push_back(_corrBoundaries.rearRight);
}

void SweptPathCreator::transform_trajectory_to_vehicle_boundaries(const tod_msgs::Trajectory& trajectory) {
    _corrBoundaries.clear();
    geometry_msgs::Point initialPoint = trajectory.points.at(0).pose.pose.position;

    int indexFirstStandstillPose = std::distance(trajectory.points.begin(),
        find_if(trajectory.points.begin(), trajectory.points.end(),
            [](const auto& n) { return n.twist.twist.linear.x == 0; }));

    if (indexFirstStandstillPose >= trajectory.points.size()) // end() points to past-last-element
        indexFirstStandstillPose = trajectory.points.size()-1;

    std::vector<double> accumulatedDistance = tod_helper::Trajectory::accumulated_distance(trajectory);

    std::for_each(accumulatedDistance.begin(), accumulatedDistance.end()-1, [=](auto &n){
        n = n - accumulatedDistance.at(indexFirstStandstillPose) - _corrParams->longitudinalOffset; });

    int lastCorridorPose = std::distance(accumulatedDistance.begin(),
        find_if(accumulatedDistance.begin(), accumulatedDistance.end(),
            [](const auto& n) { return n >= 0; }));

    if (lastCorridorPose >= accumulatedDistance.size()) // end() points to past-last-element
        lastCorridorPose = accumulatedDistance.size()-1;

    geometry_msgs::Point firstStandstillPose = trajectory.points.at(0).pose.pose.position;

    for (int i = 0; i< trajectory.points.size(); i++) {
        if (i < lastCorridorPose || i == 0) {
            double yaw = tod_helper::Geometry::get_yaw_from_quaternion(trajectory.points.at(i).pose.pose.orientation);
            geometry_msgs::Point32 point;
            tod_helper::Vehicle::Model::calc_global_pose_from_local(trajectory.points.at(i).pose.pose.position,
                yaw, _vehBoundaries.frontLeft, point);
            _corrBoundaries.frontLeft.push_back(point);
            tod_helper::Vehicle::Model::calc_global_pose_from_local(trajectory.points.at(i).pose.pose.position,
                yaw, _vehBoundaries.frontRight, point);
            _corrBoundaries.frontRight.push_back(point);
            tod_helper::Vehicle::Model::calc_global_pose_from_local(trajectory.points.at(i).pose.pose.position,
                yaw, _vehBoundaries.rearAxleRight, point);
            _corrBoundaries.rearAxleRight.push_back(point);
            tod_helper::Vehicle::Model::calc_global_pose_from_local(trajectory.points.at(i).pose.pose.position,
                yaw, _vehBoundaries.rearAxleLeft, point);
            _corrBoundaries.rearAxleLeft.push_back(point);
        }
    }
    double yaw = tod_helper::Geometry::get_yaw_from_quaternion(trajectory.points.at(0).pose.pose.orientation);
    tod_helper::Vehicle::Model::calc_global_pose_from_local(trajectory.points.at(0).pose.pose.position,
        yaw, _vehBoundaries.rearLeft, _corrBoundaries.rearLeft);
    tod_helper::Vehicle::Model::calc_global_pose_from_local(trajectory.points.at(0).pose.pose.position,
        yaw, _vehBoundaries.rearRight, _corrBoundaries.rearRight);
}
