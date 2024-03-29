// Copyright 2020 Simon Hoffmann
#pragma once

#include <ros/ros.h>
#include <ros/debug.h>
#include <vector>
#include <algorithm>
#include <string>
#include <memory>
#include <ros/console.h>
#include <tod_msgs/PrimaryControlCmd.h>
#include <tod_msgs/Status.h>
#include <tod_msgs/Trajectory.h>
#include <tod_msgs/TrajectoryPoint.h>
#include <nav_msgs/Odometry.h>
#include <tod_helper/vehicle/Model.h>
#include <tod_helper/geometry/Helpers.h>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tod_pure_pursuit/PurePursuit.h"
#include "tod_core/VehicleParameters.h"
#include "tod_pure_pursuit/ppLog.h"

class PathTrackingControl {
public:
    explicit PathTrackingControl(ros::NodeHandle& nodeHandle);
    ~PathTrackingControl() { }
    void run();

private:
    ros::NodeHandle& _nh;
    std::string _nodeName;

    //subscriber
    ros::Subscriber _subTrajectory, _subOdometry, _subStatus;

    tf2_ros::Buffer _tfBuffer;
    tf2_ros::TransformListener _transformListener;

    //callbacks
    void callback_trajectory(const tod_msgs::Trajectory& trajectory);
    void callback_odometry(const nav_msgs::Odometry& odometry);
    void callback_status_msg(const tod_msgs::Status &msg);
    geometry_msgs::TransformStamped get_transform(const std::string& srcFrame, const std::string& trgFrame);

    // publisher
    ros::Publisher _pubControlCmd, _pubLogging;

    // feedback
    tod_msgs::PrimaryControlCmd _primaryControlCommand;

    void ros_init();
    void publish_logging_output();

    //variables
    uint8_t _desiredControlMode{tod_msgs::Status::CONTROL_MODE_NONE};
    uint8_t _actualControlMode{tod_msgs::Status::CONTROL_MODE_NONE};
    tod_msgs::Trajectory _trajectory; //Ptr more performant?

    geometry_msgs::PoseStamped _poseRearAxle;
    std::unique_ptr<PurePursuit> _purePursuit;
    double _actualSpeed{0};
    std::shared_ptr<tod_core::VehicleParameters> _vehParams;
};
