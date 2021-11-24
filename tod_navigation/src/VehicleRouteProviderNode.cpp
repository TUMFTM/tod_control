// Copyright 2021 Feiler

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "XmlRouteParser.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "RouteParserNode");
    ros::NodeHandle nodeHandle;
    ros::Publisher pubRouteMsg = nodeHandle.advertise<nav_msgs::Path>("route", 1);

    XmlRouteParser xmlRouteParser;
    nav_msgs::Path route;
    ros::Rate rate(10);
    std::string pathToXml;
    std::string newPathToXml;

    while (ros::ok()) {
        nodeHandle.getParam(ros::this_node::getName() + "/pathToRoute", newPathToXml);
        if ( newPathToXml != pathToXml ) {
            pathToXml = newPathToXml;
            xmlRouteParser.setPathToXml(newPathToXml);
            try {
                xmlRouteParser.parse();
                route = xmlRouteParser.getRoute();
            } catch(const std::exception& e) {
                ROS_ERROR("%s: %s", e.what(), newPathToXml.c_str());
                route.poses.clear();
            }
        }
        pubRouteMsg.publish(route);
        rate.sleep();
    }
    return 0;
}
