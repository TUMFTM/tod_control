// Copyright 2021 Feiler

/**
 * This node listens to a route and publishes a trajectory with fixed velocity.
 * The velocity is loaded from the param server via route_to_path_config.yaml
**/

#include "ros/ros.h"
#include "RouteToPath.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "VehicleRouteToPath");
    ros::NodeHandle nodeHandle;
    int seconds;
    if (!nodeHandle.getParam(ros::this_node::getName() +
            "/secondsToWaitForTFTreeToBeBuilt", seconds)) {
        seconds = 4;
    }
    sleep(seconds);

    RouteToPath routeToPath;
    ros::Rate rate(10);
    while ( ros::ok() ) {
        ros::spinOnce();
        routeToPath.process();
        rate.sleep();
    }
    return 0;
}
