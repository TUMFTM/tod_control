// Copyright 2021 Feiler

#include "tod_network/tod_sender.h"
#include <nav_msgs/Path.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "VehicleRouteSender");
    ros::NodeHandle n;
    tod_network::Sender<nav_msgs::Path> sender(n, true);
    std::string nodeName = ros::this_node::getName();
    sender.add_processer("/Vehicle/Navigation/route",
                         tod_network::OperatorPorts::RX_ROUTE);
    sender.run();
    return 0;
}
