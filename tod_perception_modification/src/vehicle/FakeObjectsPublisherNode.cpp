// Copyright 2021 Feiler

/**
 * This node publishes objects, that are provided by *.yaml file to
 * the param server.
**/

/*
Node needs:
    - tf tree (odom and transform to lidar)

Node uses:
    - yaml with fake object
    - objects from topic
*/

#include "ros/ros.h"
#include <memory>
#include "FakeObjectsPublisher.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "FakeObjectsPublisher");
    ros::NodeHandle nodeHandle;
    int seconds;
    if (!nodeHandle.getParam(ros::this_node::getName() +
            "/secondsToWaitForTFTreeToBeBuilt", seconds)) {
        seconds = 4;
    }
    sleep(seconds);

    std::unique_ptr<FakeThingsPublisherAbstract> fakeObjectsPublisher =
        std::make_unique<FakeObjectsPublisher>();
    fakeObjectsPublisher->initFromParamServer(ros::this_node::getName());
    ros::Rate rate(10);
    while ( ros::ok() ) {
        ros::spinOnce();
        if (fakeObjectsPublisher->isCreateMode()) {
            fakeObjectsPublisher->create();
            fakeObjectsPublisher->publish();
        }
        rate.sleep();
    }
    return 0;
}
