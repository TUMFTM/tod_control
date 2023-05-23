// Copyright 2020 Andreas Schimpe
#include <ros/ros.h>
#include "ModelPredictiveController.h"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "VehicleSharedController");
    ros::NodeHandle nodeHandle;
    tod_shared_control::SharedController controller(nodeHandle);
    controller.run();
    return 0;
}
