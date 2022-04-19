// Copyright 2020 Simon Hoffmann
#include <ros/ros.h>
#include "PathTrackingControl.h"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "PathTrackingControl");
    ros::NodeHandle nodeHandle;
    PathTrackingControl controller(nodeHandle);
    controller.run();
    return 0;
}
