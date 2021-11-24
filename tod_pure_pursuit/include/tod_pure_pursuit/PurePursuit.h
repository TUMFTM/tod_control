// Copyright 2020 Simon Hoffmann
#pragma once

#include <ros/ros.h>
#include <ros/debug.h>
#include <vector>
#include <string>
#include <algorithm>
#include <ros/console.h>
#include <tod_msgs/PrimaryControlCmd.h>
#include <tod_msgs/Trajectory.h>
#include <tod_msgs/TrajectoryPoint.h>
#include <tod_helper/vehicle/Model.h>
#include <tod_helper/geometry/Helpers.h>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tod_helper/vehicle/Parameters.h"
#include "tod_helper/trajectory/Helpers.h"
#include <nav_msgs/Path.h>

struct PurePursuitConfig {
    PurePursuitConfig() = default;
    explicit PurePursuitConfig(ros::NodeHandle& nh) {
        load_from_parameter_workspace(nh);
    }
    void load_from_parameter_workspace(ros::NodeHandle& nh) {
        if (!nh.getParam(ros::this_node::getName() + "/lookaheadRatio", lookaheadRatio))
            ROS_ERROR("%s: Could not get parameter look lookaheadRatio - using %f",
                    ros::this_node::getName().c_str(), lookaheadRatio);

        if (!nh.getParam(ros::this_node::getName() + "/minLookaheadDistance", minLookahead))
            ROS_ERROR("%s: Could not get parameter ninLookaheadDistance - using %f",
                    ros::this_node::getName().c_str(), minLookahead);

        if (!nh.getParam(ros::this_node::getName() + "/rearAxleFrameId", rearAxleFrameId))
            ROS_ERROR("%s: Could not get parameter rearAxleFrameId - using %s",
                    ros::this_node::getName().c_str(), rearAxleFrameId.c_str());
    }
    double minLookahead{4.0};
    double lookaheadRatio{0.5}; // Todo: ceck if in reasonable range (<10)
    double lookaheadDistance{0.0};
    std::string rearAxleFrameId{"rear_axle_footprint"};

    void set_lookahead_distance(const double velocity) {
        lookaheadDistance =  (velocity * lookaheadRatio) > minLookahead ?
            (velocity * lookaheadRatio) : minLookahead;
    }
};

class PurePursuit {
public:
    explicit PurePursuit(ros::NodeHandle& nodeHandle);
    ~PurePursuit() { }
    bool calc_control_command(const tod_msgs::TrajectoryConstPtr trajectory,
        const geometry_msgs::PoseStamped& poseRearAxle, tod_msgs::PrimaryControlCmd& cmd,
        const double _currentVelocity);
    std::string get_rearAxleFrameId();
    int find_next_point(const tod_msgs::TrajectoryConstPtr _trajectory,
        const geometry_msgs::PoseStamped& _poseRearAxle);
    int get_closest_point(const tod_msgs::TrajectoryConstPtr trajectory,
        const geometry_msgs::PoseStamped& poseRearAxle);
    double calc_curvature(const geometry_msgs::Point &target, const geometry_msgs::PoseStamped& poseRearAxle);

private:
    double _desiredSpeed{0};
    PurePursuitConfig _ppConfig;
    tod_helper::Vehicle::Parameters _vehParam;
};
