// Copyright 2021 Andreas Schimpe
#include "CommonFeedback.h"

namespace tod_shared_control {

CommonFeedback::CommonFeedback(ros::NodeHandle &nodeHandle) :
    _nh{nodeHandle},
    _nn{ros::this_node::getName()} {
    std::string statusTopic{"/Vehicle/Manager/status_msg"};
    _subStatus = _nh.subscribe<tod_msgs::Status>(statusTopic, 1, [&](const tod_msgs::StatusConstPtr &msg) {
        _status = *msg;
    });
    _status.tod_status = tod_msgs::Status::TOD_STATUS_IDLE;
    _status.operator_control_mode = tod_msgs::Status::CONTROL_MODE_NONE;

    std::string vehDataTopic{"/Vehicle/VehicleBridge/vehicle_data"};
    _subVehData = _nh.subscribe<tod_msgs::VehicleData>(
        vehDataTopic, 1, [&](const tod_msgs::VehicleDataConstPtr &msg) {
        _vehicleData = msg;
    });

    std::string dirCtrlTopic{"/Vehicle/CommandCreation/primary_control_cmd"};
    _subCtrlCmd = _nh.subscribe<tod_msgs::PrimaryControlCmd>(
        dirCtrlTopic, 1, [&](const tod_msgs::PrimaryControlCmdConstPtr &msg) {
            _primaryCtrlCmd = msg;
        });

    std::string odomTopic{"/Vehicle/VehicleBridge/odometry"};
    _subOdom = _nh.subscribe<nav_msgs::Odometry>(
        odomTopic, 1, [&] (const nav_msgs::OdometryConstPtr &msg) {
            _odometry = msg;
        });
}

}; // namespace tod_shared_control
