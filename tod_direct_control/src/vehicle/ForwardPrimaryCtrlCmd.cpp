// Copyright 2020 Simon Hoffmann
#include "ros/ros.h"
#include "tod_msgs/Status.h"
#include "tod_msgs/PrimaryControlCmd.h"

uint8_t _ctrlMode;
ros::Publisher pubControlMsg;

bool inDirectControlMode() { return _ctrlMode == tod_msgs::Status::CONTROL_MODE_DIRECT; }

void callback_direct_control(const tod_msgs::PrimaryControlCmd& msg) {
    if ( inDirectControlMode() ) {
        pubControlMsg.publish(msg);
    }
}
void callback_status(const tod_msgs::Status& msg) {
    _ctrlMode = msg.vehicle_control_mode;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ForwardPrimaryCtrlCmd");
    ros::NodeHandle nodeHandle;
    std::vector<ros::Subscriber> listOfSubscribers;
    listOfSubscribers.push_back(nodeHandle.subscribe("status_msg", 1, callback_status));
    listOfSubscribers.push_back(nodeHandle.subscribe("primary_cmd_in", 1, callback_direct_control));
    pubControlMsg = nodeHandle.advertise<tod_msgs::PrimaryControlCmd>("primary_cmd_out", 1);
    ros::spin();
    return 0;
}
