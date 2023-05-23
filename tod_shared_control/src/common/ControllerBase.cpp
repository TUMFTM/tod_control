// Copyright 2021 Andreas Schimpe
#include "ControllerBase.h"

namespace tod_shared_control {

ControllerBase::ControllerBase(ros::NodeHandle &nodeHandle) :
    _nh(nodeHandle),
    _nn(ros::this_node::getName()),
    _fb{nodeHandle},
    _pubCtrlCmd{_nh.advertise<tod_msgs::PrimaryControlCmd>("primary_control_cmd", 1)},
    _transformListener{tf2_ros::TransformListener(_tfBuffer)} { }

void ControllerBase::check_cycle_time(const ros::Rate &rate) {
    if (rate.cycleTime().toSec() > rate.expectedCycleTime().toSec()) {
        ROS_WARN("%s: cycle time (%.1f ms) exceeded expected cycle time (%.1f ms)", _nn.c_str(),
                 rate.cycleTime().toSec()*1e3, rate.expectedCycleTime().toSec()*1e3);
    }
}

void ControllerBase::publish_ctrl_cmd(tod_msgs::PrimaryControlCmd &ctrlCmd) {
    ctrlCmd.header.stamp = ros::Time::now();
    _pubCtrlCmd.publish(ctrlCmd);
}

geometry_msgs::TransformStamped ControllerBase::get_tf(const std::string &sourceFrame, const std::string& targetFrame) {
    if (_tfBuffer.canTransform(targetFrame, sourceFrame, ros::Time::now()))
        return _tfBuffer.lookupTransform(targetFrame, sourceFrame, ros::Time::now());
    ROS_WARN("%s: cannot get transform from %s to %s for log msg", _nn.c_str(),
             sourceFrame.c_str(), targetFrame.c_str());
    return geometry_msgs::TransformStamped();
}

} // namespace tod_shared_control
