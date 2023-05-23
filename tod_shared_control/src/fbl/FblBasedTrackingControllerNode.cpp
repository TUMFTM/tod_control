// Copyright 2020 Andreas Schimpe
#include <ros/ros.h>
#include "FblBasedTrackingController.h"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "FblBasedTrackingController");
    ros::NodeHandle nodeHandle;
    tod_shared_control::FblBasedTrackingController controller(nodeHandle);
    controller.run();
    return 0;
}
