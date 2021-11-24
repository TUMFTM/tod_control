// Copyright 2021 Feiler

#include "ros/ros.h"
#include "VehiclePerceptionModification.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "VehiclePerceptionModification");
    ros::NodeHandle nodeHandle;

    VehiclePerceptionModification vehiclePerceptionModification;
    ros::Rate rate(10);
    while ( ros::ok() ) {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
