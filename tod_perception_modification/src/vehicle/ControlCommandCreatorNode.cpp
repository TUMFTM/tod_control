// Copyright 2021 Feiler

/**
 * This node acts as a gate.
 * It listens to the control command output of pure pursuit and to the current
 * mode (AV or teleoperation). If mode == AV, than the control commands are forwarded.
 * Otherwise, no control commands are published by this node.
**/

#include "ros/ros.h"
#include "ControlCommandCreator.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "VehicleControlCommandCreator");
    ros::NodeHandle nodeHandle;
    ControlCommandCreator controlCommandCreator;
    ros::Rate rate(100);
    while ( ros::ok() ) {
        ros::spinOnce();
        controlCommandCreator.publish();
        rate.sleep();
    }
    return 0;
}
