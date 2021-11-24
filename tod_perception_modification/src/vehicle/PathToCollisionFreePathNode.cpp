// Copyright 2021 Feiler

/**
 * Node listens to path, objects and gridmap and publishes a trajectory.
 * If objects or gridmap reflections lay within the current path,
 * the trajectory brakes into standstill.
**/ 

#include "ros/ros.h"
#include "PathToCollisionFreePath.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "VehiclePathToCollisionFreePath");
    ros::NodeHandle nodeHandle;

    PathToCollisionFreePath pathToCollisionFreePath;
    ros::Rate rate(10);
    while ( ros::ok() ) {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
