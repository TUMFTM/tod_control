// Copyright 2021 Feiler

// receives and forwards appropriate operator commands
// (velocity, status, area)

#include "ros/ros.h"
#include "VehicleCommunicator.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "VehicleCommunicatorNode");
    ros::NodeHandle nodeHandle;
    VehicleCommunicator vehicleCommunicator;
    ros::Rate rate(10);
    while ( ros::ok() ) {
        ros::spinOnce();
        vehicleCommunicator.publish();
        rate.sleep();
    }
    return 0;
}
