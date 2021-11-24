// Copyright 2021 Feiler

#include "tod_network/tod_receiver.h"
#include <nav_msgs/Path.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "OperatorRouteReceiver");
    ros::NodeHandle n;
    tod_network::Receiver<nav_msgs::Path> receiver(n);
    std::string nodeName = ros::this_node::getName();
    receiver.add_processer("/Operator/Navigation/route",
                           tod_network::OperatorPorts::RX_ROUTE);
    receiver.run();
    return 0;
}
