// Copyright 2021 Feiler
#pragma once
#include "ros/ros.h"
#include "FakeThingsPublisherAbstract.h"
#include <memory>
#include <string>
#include <vector>
#include "Timer.h"
#include "grid_map_msgs/GridMap.h"
#include <grid_map_ros/grid_map_ros.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "nav_msgs/Odometry.h"

class FakeCellsPublisher : public FakeThingsPublisherAbstract {
public:
    FakeCellsPublisher();
    virtual void initFromParamServer(const std::string& nodeName);
    virtual bool isCreateMode();
    virtual void create();
    virtual void publish();
    void callbackGridMap(const grid_map_msgs::GridMapConstPtr msg);
    void processArrivedOdomFrame(const nav_msgs::Odometry& msg);

private:
    ros::NodeHandle nodeHandle;
    Mode currentMode;
    std::unique_ptr<Timer> timer;
    grid_map::GridMap gridMap;
    std::vector<ros::Subscriber> subscribers;
    ros::Publisher pubGridMap;
    std::vector<grid_map::Position> reflections;
    std::string reflectionsFrameId;

    double gridMapWidthInM;
    double gridMapHeightInM;
    double gridMapCellSizeInM;
    double mapBehindVehicleInM;
    double lidarHeight;
    double occupied;
    double free;
    std::string gridMapFrameId { "ftm" };
    tf2_ros::Buffer itsTf2Buffer;
    tf2_ros::TransformListener itsTf2Listener;

    void lookUpAndAddReflection(const int reflectionNumber, const std::string& nodeName);
    void appendFakeReflectionsToGridMap();
    void initGridMap();
    void loadParamsFromParamServer();
    void printStandardRosWarn(const std::string& paramName);
    void moveGridMapAccordingToOdometry(const nav_msgs::Odometry& msg);
    void calcAndPrintBitRate(const grid_map_msgs::GridMap& map,
            int nodeFrequency);

    template <class T>
    void loadParam(const std::string& paramName, T& parameter, const T defaultValue) {
        if ( !nodeHandle.getParam(ros::this_node::getName() + "/" + paramName, parameter) ) {
            printStandardRosWarn(paramName);
            parameter = defaultValue;
        }
    }
};
