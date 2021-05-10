// Copyright 2020 Simon Hoffmann
#include "ros/ros.h"
#include "CommandCreator.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "CommandCreator");
    ros::NodeHandle nodeHandle;
    CommandCreator commandCreation(nodeHandle);
    commandCreation.run();
    return 0;
}
