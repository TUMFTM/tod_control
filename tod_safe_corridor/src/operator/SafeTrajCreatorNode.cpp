// Copyright 2020 Simon Hoffmann
#include "ros/ros.h"
#include "SafeTrajCreator.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "SafeTrajCreator");
    ros::NodeHandle n;
    SafeTrajCreator trajCreator(n);
    trajCreator.run();

    return 0;
}
