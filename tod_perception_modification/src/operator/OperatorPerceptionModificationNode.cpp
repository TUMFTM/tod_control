// Copyright 2020 Feiler

#include "ros/ros.h"
#include "OperatorPerceptionModification.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "OperatorPerceptionModification");
    ros::NodeHandle nodeHandle;

    OperatorPerceptionModification perceptionModification(nodeHandle);
    ros::Rate rate(30);
    while (ros::ok()) {
        ros::spinOnce();
        perceptionModification.process();
        rate.sleep();
    }
    return 0;
}
