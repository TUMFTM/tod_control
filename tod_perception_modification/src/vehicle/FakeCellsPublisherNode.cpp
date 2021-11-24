// Copyright 2021 Feiler

/**
 * This node publishes lidar reflections, that are provided by *.yaml file to
 * the param server.
**/

#include "ros/ros.h"
#include <memory>
#include "FakeCellsPublisher.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "FakeCellsPublisher");
    ros::NodeHandle nodeHandle;

    std::unique_ptr<FakeThingsPublisherAbstract> fakeCellsPublisher =
        std::make_unique<FakeCellsPublisher>();
    fakeCellsPublisher->initFromParamServer(ros::this_node::getName());
    ros::Rate rate(10);
    while ( ros::ok() ) {
        ros::spinOnce();
        if ( fakeCellsPublisher->isCreateMode() ) {
            fakeCellsPublisher->create();
            fakeCellsPublisher->publish();
        }
        rate.sleep();
    }
    return 0;
}
