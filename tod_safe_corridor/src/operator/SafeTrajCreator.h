// Copyright 2020 Simon Hoffmann
#pragma once
#include <memory>
#include <string>
#include <algorithm>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include "tod_helper/vehicle/Model.h"
#include "tod_helper/colored_polygon/Helpers.h"
#include "tod_helper/geometry/Helpers.h"
#include "tod_msgs/PrimaryControlCmd.h"
#include "tod_msgs/Status.h"
#include "tod_msgs/Trajectory.h"
#include "tod_msgs/ColoredPolygon.h"
#include "std_msgs/ColorRGBA.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PolygonStamped.h"
#include "tod_safe_corridor/SweptPathCreator.h"
#include "tod_core/VehicleParameters.h"
#include "tod_safe_corridor/PlannerFactory.h"
#include "tod_safety_monitoring/GateState.h"
#include "tod_safety_monitoring/Definitions.h"
#include "tod_safety_monitoring/Utils.h"

using SMState = tod_safety_monitoring::StateMachineState;

class PathCreatorParams{
public:
    PathCreatorParams() = default;
    explicit PathCreatorParams(ros::NodeHandle& nh) {
        load_from_parameter_workspace(nh);
    }
    void load_from_parameter_workspace(ros::NodeHandle& nh) {
        if (!nh.getParam(ros::this_node::getName() + "/segmentLength", segmentLength))
            ROS_ERROR("%s: Could not get parameter /segmentLength - using %f",
                    ros::this_node::getName().c_str(), segmentLength);

        if (!nh.getParam(ros::this_node::getName() + "/lookaheadRatio", lookaheadRatio))
            ROS_ERROR("%s: Could not get parameter /lookaheadRatio - using %f",
                    ros::this_node::getName().c_str(), lookaheadRatio);

        if (!nh.getParam(ros::this_node::getName() + "/minLookaheadDistance", minLookaheadDistance))
            ROS_ERROR("%s: Could not get parameter /minLookaheadDistance - using %f",
                    ros::this_node::getName().c_str(), minLookaheadDistance);

        if (!nh.getParam(ros::this_node::getName() + "/frame_odom", frame_id))
            ROS_ERROR("%s: Could not get parameter /frame_odom - using %s",
                    ros::this_node::getName().c_str(), frame_id.c_str());

        if (!nh.getParam(ros::this_node::getName() + "/frame_rear_axle_footprint", child_frame_id))
            ROS_ERROR("%s: Could not get parameter /frame_rear_axle_footprint - using %s",
                    ros::this_node::getName().c_str(), child_frame_id.c_str());
    }
    double get_segment_length() { return segmentLength; }
    double get_lookahead_distance(const double velocity) {
        return (velocity * lookaheadRatio) > minLookaheadDistance ?
            (velocity * lookaheadRatio) : minLookaheadDistance;
    }
    std::string get_frame_id() { return frame_id; }
    std::string get_child_frame_id() { return child_frame_id; }

private:
    double segmentLength{0.3};
    double lookaheadRatio{0.0};
    double minLookaheadDistance{0.0};
    std::string frame_id {"ftm"};
    std::string child_frame_id {"rear_axle_footprint"};
};

class SafeTrajCreator {
public:
    explicit SafeTrajCreator(ros::NodeHandle &nodeHandle);
    void run();

private:
    ros::NodeHandle _nh;
    ros::Subscriber _inputSubs, _poseSubs, _statusSubs, _vGateStateSubs, _oGateStateSubs;;
    ros::Publisher _trajectoryPub, _corridorPub;
    tf2_ros::Buffer _tfBuffer;
    tf2_ros::TransformListener _transformListener;

    // Parameter sets
    std::shared_ptr<tod_core::VehicleParameters> _vehParams;
    std::shared_ptr<SweptPathParams> _corridorParams;
    std::unique_ptr<SpeedPlannerParams> _speedPlannerParams;
    std::unique_ptr<PathCreatorParams> _trajParams;

    std::unique_ptr<LateralPlanner> _latPlanner;
    std::unique_ptr<LongitudinalPlanner> _longPlanner;
    std::unique_ptr<SweptPathCreator> _sweptPathCreator;
    uint8_t _ctrlMode;
    tod_msgs::PrimaryControlCmdConstPtr _ctrlCmd;
    SMState _oGate;
    SMState _vGate;
    bool _poseIsSet{false};
    bool _newControlCommand{false};

    bool in_safe_corridor_control() { return _ctrlMode == tod_msgs::Status::CONTROL_MODE_SAFECORRIDOR; }
    double calc_max_path_length(const double velocity, const double offset_to_deceleration);
    double calc_offset_to_deceleration(const double velocity);
    void init_ros_interface();
    void init_param_sets();

    // callbacks
    void callback_control_msg(const tod_msgs::PrimaryControlCmdConstPtr msg);
    void callback_pose(const nav_msgs::Odometry &msg);
    void callback_status(const tod_msgs::Status& msg);
    void callback_gate_state_vehicle(const tod_safety_monitoring::GateState& msg);
    void callback_gate_state_operator(const tod_safety_monitoring::GateState& msg);
};
