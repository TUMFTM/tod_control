// Copyright 2021 Feiler
#pragma once
#include "ros/ros.h"
#include "tod_msgs/Trajectory.h"
#include "grid_map_msgs/GridMap.h"
#include "tod_msgs/ObjectList.h"
#include <grid_map_ros/grid_map_ros.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <glm/glm.hpp>
#include "ObjectList.h"
#include <vector>
#include <string>

class PathToCollisionFreePath {
public:
    PathToCollisionFreePath();

private:
    ros::NodeHandle nodeHandle;
    std::vector<ros::Subscriber> subscribers;
    ros::Publisher pubCollisionFreeTrajectory;
    grid_map::GridMap inputGridMap;
    tod_msgs::ObjectList inputObjectList;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    double vehicleWidth;
    double free;
    double distanceFrontBumper;
    double distanceRearAxle;
    double bufferDistanceInM;
    double brakeAcceleration;
    double radiusBuffer;

    void callbackInputTrajectory(const tod_msgs::TrajectoryConstPtr msg);
    void callbackInputGridMap(const grid_map_msgs::GridMap& msg);
    void callbackInputObjectList(const tod_msgs::ObjectList& msg);
    void turnDebuggingOn();
    void addInputObjectListToInputGridMap();
    bool getTransformToTargetFromSource(const std::string& targetFrame,
        const std::string& sourceFrame,
        geometry_msgs::TransformStamped& transform);
    void fillGridWithFloorPolygon(const grid_map::Polygon& objectAsPolygon);
    bool trajectoryCollidesWithGridMapData(
        const grid_map::GridMap& gridMap,
        const tod_msgs::Trajectory& trajectory,
        int& trajectoryIndex, grid_map::Index& gridMapPositionIndex);
    void loadParamsFromParamServer();
    void printStandardRosWarn(const std::string& paramName);
    int getIndexOfDesiredRearAxleStopPoint(const tod_msgs::Trajectory& outTrajectory,
        const int index, grid_map::Index& gridMapPositionIndex);
    void tellTheUserIfChildFrameIdIsNotCorrect(const tod_msgs::Trajectory& outTrajectory);
    void modifyTrajectoryToBrakeAppropriately(
        tod_msgs::Trajectory& outTrajectory,
        const int indexOfDesiredTrajectoryStopPoint);
    void tellTheUserEveryXSecondThatSomethingWouldCollide(const int seconds);
    template <class T>
    void loadParam(const std::string& paramName, T& parameter, const T defaultValue) {
        if ( !nodeHandle.getParam(ros::this_node::getName() + "/" + paramName, parameter) ) {
            printStandardRosWarn(paramName);
            parameter = defaultValue;
        }
    }
};

