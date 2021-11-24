// Copyright 2021 Feiler
#pragma once
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "tod_helper/trajectory/StraightTrajectory.h"
#include <string>
#include <vector>
#include <memory>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "std_msgs/Bool.h"

class RouteToPath {
public:
    RouteToPath();
    void callbackRoute(const nav_msgs::PathConstPtr msg);
    void process();

private:
    std::vector<ros::Subscriber> subscribers;
    ros::Publisher pubPath;
    ros::NodeHandle nodeHandle;
    std::unique_ptr<tod_helper::Trajectory::TodTrajectoryCreator> trajectoryCreator;
    tf2_ros::Buffer tf2Buffer;
    tf2_ros::TransformListener tf2Listener;
    double incrementFactor {0.0};
    double velocityIncrement;
    nav_msgs::Path inputPath;
    double newVelocity;

    tod_msgs::Trajectory transformRouteToPathWithAppropriateSpacingsForPurePursuit(
        const nav_msgs::Path& msg);
    bool getCurrentPositionOf_In(const std::string& sourceFrame,
        const std::string& targetFrame, geometry_msgs::PoseStamped& outPose);
    double calcDistanceBetweenPoints(const tod_msgs::TrajectoryPoint& trajPoint,
        const geometry_msgs::PoseStamped& rearAxlePoseInFtmFrame);
    int getIndexOfClosestPoint(const tod_msgs::Trajectory& trajectory,
        const geometry_msgs::PoseStamped& rearAxlePoseInFtmFrame,
        const double& terminationCriterion);
    tod_msgs::Trajectory takeOnlyNPointsFromProvidedIndex(
        const tod_msgs::Trajectory& trajectory,
        const int numberOfPoints, const int indexOfSmallestDistance);

    void callbackOperatorIncrementVelocity(const std_msgs::Bool& msg);
    void adaptVelocityAccordingToOperatorWishes(tod_msgs::Trajectory& trajectory);
};
