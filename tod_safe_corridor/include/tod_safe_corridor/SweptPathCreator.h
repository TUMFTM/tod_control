// Copyright 2020 Simon Hoffmann
#pragma once
#include <ros/ros.h>
#include <memory>
#include <ros/debug.h>
#include <algorithm>
#include <vector>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tod_helper/vehicle/Model.h>
#include "tod_msgs/Trajectory.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PolygonStamped.h"
#include "tod_helper/geometry/Helpers.h"
#include "tod_helper/trajectory/Helpers.h"
#include "tod_core/VehicleParameters.h"

class SweptPathParams{
public:
    SweptPathParams() = default;
    explicit SweptPathParams(ros::NodeHandle& nh) {
        load_from_parameter_workspace(nh);
    }
    SweptPathParams(const double bufferFront, const double bufferSide, const double longitudinalOffset) :
        bufferFront(bufferFront), bufferSideStat(bufferSide), longitudinalOffset(longitudinalOffset) { }

    void load_from_parameter_workspace(ros::NodeHandle& nh) {
        if (!nh.getParam(ros::this_node::getName() + "/corridorBufferFront", bufferFront))
            ROS_ERROR("%s: Could not get parameter /corridorBufferFront - using %f",
                    ros::this_node::getName().c_str(), bufferFront);

        if (!nh.getParam(ros::this_node::getName() + "/corridorBufferSide", bufferSideStat))
            ROS_ERROR("%s: Could not get parameter /corridorBufferSide - using %f",
                    ros::this_node::getName().c_str(), bufferSideStat);

        if (!nh.getParam(ros::this_node::getName() + "/corridorLongitudinalOffset", longitudinalOffset))
                ROS_ERROR("%s: Could not get parameter /corridorLongitudinalOffset - using %f",
                        ros::this_node::getName().c_str(), longitudinalOffset);
    }

    double bufferFront{0.1};
    double bufferSideStat{0.1};
    double longitudinalOffset{3.0};
};

struct VehicleBoundaries{
    geometry_msgs::Point32 frontLeft;
    geometry_msgs::Point32 frontRight;
    geometry_msgs::Point32 rearLeft;
    geometry_msgs::Point32 rearRight;
    geometry_msgs::Point32 rearAxleLeft;
    geometry_msgs::Point32 rearAxleRight;
};

struct CorridorBoundaries{
    std::vector<geometry_msgs::Point32> frontLeft;
    std::vector<geometry_msgs::Point32> frontRight;
    std::vector<geometry_msgs::Point32> rearAxleLeft;
    std::vector<geometry_msgs::Point32> rearAxleRight;
    geometry_msgs::Point32 rearLeft;
    geometry_msgs::Point32 rearRight;
    void clear(){
        frontLeft.clear();
        frontRight.clear();
        rearAxleLeft.clear();
        rearAxleRight.clear();
    }
};

class SweptPathCreator {
public:
    SweptPathCreator(std::shared_ptr<tod_core::VehicleParameters> vehParams, double bufferFront,
        double bufferSide, double longitudinalOffset);
    SweptPathCreator(std::shared_ptr<tod_core::VehicleParameters> vehParams,
        std::shared_ptr<SweptPathParams> sweptPathParams);
    void create_corridor(geometry_msgs::PolygonStamped& corridor,
            const tod_msgs::Trajectory& trajectory, const double curvature);
    void create_corridor(geometry_msgs::PolygonStamped& corridor,
            const tod_msgs::Trajectory& trajectory);
    void apply_dynamic_side_buffer(const double bufferSideDyn);

private:
    void init_vehicle_boundaries();
    std::shared_ptr<tod_core::VehicleParameters> _vehParams;
    std::shared_ptr<SweptPathParams> _corrParams;
    VehicleBoundaries _vehBoundaries;
    CorridorBoundaries _corrBoundaries;
    void on_curve_left(geometry_msgs::PolygonStamped& corridor);
    void on_curve_right(geometry_msgs::PolygonStamped& corridor);
    void transform_trajectory_to_vehicle_boundaries(const tod_msgs::Trajectory& trajectory);
};
