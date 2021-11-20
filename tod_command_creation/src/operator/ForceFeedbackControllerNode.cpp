// Copyright 2021 Schimpe
#include <ros/ros.h>
#include "ForceFeedbackController.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "ForceFeedbackController");
    ros::NodeHandle nh;
    ForceFeedbackController controller(nh);
    controller.run();
    return 0;
}
