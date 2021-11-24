// Copyright Feiler 2021

#include "ros/ros.h"
#include <Polyline2D/Polyline2D.h>
#include "tod_msgs/Mesh.h"
#include "nav_msgs/Path.h"

ros::Publisher pubRouteAsMesh;

std::vector<crushedpixel::Vec2> convertRouteMsgToVec2(const nav_msgs::Path& route) {
    std::vector<crushedpixel::Vec2> inputPoints;
    for ( auto iterator = route.poses.begin();
    iterator != route.poses.end(); ++iterator ) {
        crushedpixel::Vec2 tmpVec2;
        tmpVec2.x = iterator->pose.position.x;
        tmpVec2.y = iterator->pose.position.y;
        inputPoints.push_back(tmpVec2);
    }
    return inputPoints;
}

tod_msgs::Mesh convertVec2ToMeshMsg(const std::vector<crushedpixel::Vec2>& verticesVec2) {
    std::vector<geometry_msgs::Point> vertices;
    for ( auto iterator = verticesVec2.begin(); iterator != verticesVec2.end(); ++iterator ) {
        geometry_msgs::Point tmpPoint;
        tmpPoint.x = iterator->x;
        tmpPoint.y = iterator->y;
        tmpPoint.z = 0.0;
        vertices.push_back(tmpPoint);
    }
    tod_msgs::Mesh routeAsMesh;
    routeAsMesh.vertices = vertices;
    return routeAsMesh;
}

void addColorToMeshMsg(tod_msgs::Mesh& routeAsMesh, float r, float g, float b) {
    tod_msgs::Color color;
    color.r = r;
    color.g = g;
    color.b = b;
    for ( int iterator = 0; iterator != routeAsMesh.vertices.size(); ++iterator ) {
        routeAsMesh.colors.push_back(color);
    }
}

tod_msgs::Mesh createRouteAsMeshFromMsg(const nav_msgs::Path& route) {
    std::vector<crushedpixel::Vec2> inputPoints = convertRouteMsgToVec2(route);
    auto thickness = 1.0f;
    auto verticesVec2 = crushedpixel::Polyline2D::create(inputPoints, thickness,
        crushedpixel::Polyline2D::JointStyle::ROUND,
        crushedpixel::Polyline2D::EndCapStyle::SQUARE);
    tod_msgs::Mesh routeAsMesh = convertVec2ToMeshMsg(verticesVec2);
    addColorToMeshMsg(routeAsMesh, 0.2f, 0.6f, 0.2f);
    return routeAsMesh;
}

void routeCallback(const nav_msgs::Path& msg) {
    tod_msgs::Mesh routeAsMesh = createRouteAsMeshFromMsg(msg);
    pubRouteAsMesh.publish(routeAsMesh);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "NavigationMeshCreator");
    ros::NodeHandle nodeHandle;
    ros::Subscriber subToRoute = nodeHandle.subscribe("/route", 5, routeCallback);
    pubRouteAsMesh = nodeHandle.advertise<tod_msgs::Mesh>("/routeAsMesh", 1);

    ros::Rate rate(100);
    while ( ros::ok() ) {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
