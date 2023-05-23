// Copyright 2021 Andreas Schimpe
#include <ros/ros.h>
#include "SharedVelocityController.h"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "SharedVelocityController");
    ros::NodeHandle nodeHandle;
    tod_shared_control::SharedVelocityController controller(nodeHandle);
    controller.run();
    return 0;
}
