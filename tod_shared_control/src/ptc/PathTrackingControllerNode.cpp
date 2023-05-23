// Copyright 2020 Andreas Schimpe
#include <ros/ros.h>
#include "PathTrackingController.h"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "PathTrackingController");
    ros::NodeHandle nodeHandle;
    tod_shared_control::PathTrackingController controller(nodeHandle);
    controller.run();
    return 0;
}
