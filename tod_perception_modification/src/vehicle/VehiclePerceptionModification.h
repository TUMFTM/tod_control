// Copyright 2021 Feiler
#pragma once
#include <string>
#include <vector>
#include <map>
#include "ros/ros.h"
#include "grid_map_msgs/GridMap.h"
#include "grid_map_core/grid_map_core.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "tod_msgs/ObjectList.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/PointCloud.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "nav_msgs/Odometry.h"

class VehiclePerceptionModification {
public:
    VehiclePerceptionModification();

private:
    ros::NodeHandle nodeHandle;
    std::vector<ros::Subscriber> subscribers;
    std::map<std::string, ros::Publisher> publishers;
    bool applyModification { false };
    sensor_msgs::PointCloud area;
    grid_map::GridMap gridMapWithMarkedArea;
    nav_msgs::Odometry odometry;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    void callbackGridMap(const grid_map_msgs::GridMapConstPtr msg);
    void deleteAffectedCells(grid_map::GridMap& receivedGridMap);
    grid_map::Polygon createPolygonFromArea();
    void setCellsInAreaToZero(grid_map::GridMap& receivedGridMap,
            const grid_map::Polygon& areaAsPolygon);
    void callbackObjectList(const tod_msgs::ObjectListConstPtr msg);
    void callbackApplyModification(const std_msgs::Bool& msg);
    void callbackArea(const sensor_msgs::PointCloud& msg);
    void deleteAffectedObjects(tod_msgs::ObjectList& modifiedObjectList);
    void callbackOdometry(const nav_msgs::Odometry& msg);
    void initGridMapWithMarkedArea(const std::string& frame_id);
    nav_msgs::Odometry convertToPositionBehind(const nav_msgs::Odometry& odometry,
            const double& metersBehind);
};

