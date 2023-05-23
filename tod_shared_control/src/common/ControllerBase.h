// Copyright 2021 Andreas Schimpe
#pragma once
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tod_msgs/PrimaryControlCmd.h>
#include <string>
#include <map>
#include "CommonFeedback.h"

namespace tod_shared_control {

class ControllerBase {
public:
    explicit ControllerBase(ros::NodeHandle& nodeHandle);
    virtual ~ControllerBase() { }
    virtual void run() = 0;

    ros::NodeHandle& _nh;
    std::string _nn;
    tod_shared_control::CommonFeedback _fb;
    std::map<std::string, ros::Subscriber> _subscriberMap;
    ros::Publisher _pubCtrlCmd;
    tf2_ros::Buffer _tfBuffer;
    tf2_ros::TransformListener _transformListener;

    void check_cycle_time(const ros::Rate &rate);
    void publish_ctrl_cmd(tod_msgs::PrimaryControlCmd &ctrlCmd);
    geometry_msgs::TransformStamped get_tf(const std::string &sourceFrame, const std::string& targetFrame);
};

} // namespace tod_shared_control
