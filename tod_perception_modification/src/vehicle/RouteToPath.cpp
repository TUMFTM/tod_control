// Copyright 2021 Feiler

#include "RouteToPath.h"
#include "tod_msgs/Trajectory.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/convert.h"
#include "ros/console.h"
#include <algorithm>
#include "LoadParam.h"

RouteToPath::RouteToPath() : tf2Listener(tf2Buffer) {
    subscribers.push_back(nodeHandle.subscribe<nav_msgs::PathConstPtr>(
        "route", 5, &RouteToPath::callbackRoute, this));
    pubPath = nodeHandle.advertise<tod_msgs::Trajectory>(
        "path", 2);

    subscribers.push_back(nodeHandle.subscribe("increment_velocity", 5,
                    &RouteToPath::callbackOperatorIncrementVelocity, this));

    TodHelper::loadParam(nodeHandle, "velocity_increment", velocityIncrement, 0.833);
    trajectoryCreator = std::make_unique<tod_helper::Trajectory::StraightTrajectory>(nodeHandle);
}

void RouteToPath::callbackRoute(const nav_msgs::PathConstPtr msg) {
    inputPath = *msg;
}

void RouteToPath::process() {
    if ( inputPath.poses.empty() ) {
        return;
    }
    std::string childFrameIdOfPathPoints { "rear_axle_footprint" };
    std::string globalFrameId { "ftm" };
    tod_msgs::Trajectory trajectory =
        transformRouteToPathWithAppropriateSpacingsForPurePursuit(inputPath);
    adaptVelocityAccordingToOperatorWishes(trajectory);
    trajectoryCreator->brakeIntoStillStand(trajectory, newVelocity, -2.0,
        10);
    geometry_msgs::PoseStamped rearAxlePoseInFtmFrame;
    if ( !getCurrentPositionOf_In(childFrameIdOfPathPoints, globalFrameId,
        rearAxlePoseInFtmFrame) ) {
        return;
    }
    if ( trajectory.points.empty() ) {
        ROS_ERROR("Trajectory does not have one point at %s", ros::this_node::getName().c_str());
        return;
    }
    double terminationCriterionInMeter { 0.5 };
    int indexOfSmallestDistance = getIndexOfClosestPoint(trajectory,
        rearAxlePoseInFtmFrame, terminationCriterionInMeter);

    int numberOfPoints { 30 };
    tod_msgs::Trajectory cutTrajectory = takeOnlyNPointsFromProvidedIndex(trajectory,
        numberOfPoints, indexOfSmallestDistance);
    pubPath.publish(cutTrajectory);
}

tod_msgs::Trajectory RouteToPath::transformRouteToPathWithAppropriateSpacingsForPurePursuit(
        const nav_msgs::Path& msg) {
    std::vector<geometry_msgs::Point> tmpInputForTrajectoryCreator;
    for ( auto poseStamped : msg.poses ) {
        tmpInputForTrajectoryCreator.push_back(poseStamped.pose.position);
    }
    tod_msgs::Trajectory trajectory = trajectoryCreator->createFrom(tmpInputForTrajectoryCreator);
    return trajectory;
}

bool RouteToPath::getCurrentPositionOf_In(
        const std::string& sourceFrame, const std::string& targetFrame,
        geometry_msgs::PoseStamped& outPose) {
    static bool tellOnlyOnce {true};
    if ( !tf2Buffer.canTransform(targetFrame, sourceFrame, ros::Time(0)) ) {
        if ( tellOnlyOnce ) {
            tellOnlyOnce = false;
            ROS_ERROR("no we cannot do the transform at %s",
                ros::this_node::getName().c_str());
        }
        return false;
    } else {
        tellOnlyOnce = true;
    }
    geometry_msgs::TransformStamped rearAxleToFtm;
    try {
        rearAxleToFtm = tf2Buffer.lookupTransform(targetFrame, sourceFrame,
            ros::Time(0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s at %s", ex.what(), ros::this_node::getName().c_str());
    }
    geometry_msgs::PoseStamped rearAxlePose;
    rearAxlePose.pose.orientation.w = 1.0; // no rotation
    tf2::doTransform(rearAxlePose, outPose, rearAxleToFtm);
    return true;
}

int RouteToPath::getIndexOfClosestPoint(const tod_msgs::Trajectory& trajectory,
const geometry_msgs::PoseStamped& rearAxlePoseInFtmFrame,
const double& terminationCriterionInMeter) {
    int indexOfSmallestDistance { 0 };
    double distance;
    distance = calcDistanceBetweenPoints(trajectory.points.front(),
        rearAxlePoseInFtmFrame);
    int index { 1 };
    while ( (distance > terminationCriterionInMeter) && (index < trajectory.points.size()) ) {
        double newDistance = calcDistanceBetweenPoints(trajectory.points.at(index),
            rearAxlePoseInFtmFrame);
        if ( newDistance < distance ) {
            distance = newDistance;
            indexOfSmallestDistance = index;
        }
        ++index;
    }
    return indexOfSmallestDistance;
}

double RouteToPath::calcDistanceBetweenPoints(
        const tod_msgs::TrajectoryPoint& trajPoint,
        const geometry_msgs::PoseStamped& rearAxlePoseInFtmFrame) {
    return  std::sqrt((std::pow(rearAxlePoseInFtmFrame.pose.position.x -
                    trajPoint.pose.pose.position.x, 2) +
                    std::pow(rearAxlePoseInFtmFrame.pose.position.y -
                    trajPoint.pose.pose.position.y, 2)));
}

tod_msgs::Trajectory RouteToPath::takeOnlyNPointsFromProvidedIndex(
const tod_msgs::Trajectory& trajectory,
const int numberOfPoints, const int indexOfSmallestDistance) {
    int availablePoints = trajectory.points.size() - indexOfSmallestDistance;
    if ( availablePoints > numberOfPoints ) {
        availablePoints = numberOfPoints;
    }
    tod_msgs::Trajectory returnTrajectory;
    returnTrajectory.child_frame_id = trajectory.child_frame_id;
    returnTrajectory.header = trajectory.header;
    returnTrajectory.points.resize(availablePoints);
    std::copy(trajectory.points.begin() + indexOfSmallestDistance,
        trajectory.points.begin() + indexOfSmallestDistance + availablePoints,
        returnTrajectory.points.begin());
    ROS_DEBUG_STREAM("return trajectory from RouteToPath " << returnTrajectory);
    return returnTrajectory;
}

void RouteToPath::callbackOperatorIncrementVelocity(const std_msgs::Bool& msg) {
    ROS_ERROR("speed change is %s", msg.data ? "positive" : "negative");
    if ( msg.data ) {
        incrementFactor += 1;
    } else {
        incrementFactor -= 1;
    }
}

void RouteToPath::adaptVelocityAccordingToOperatorWishes(
tod_msgs::Trajectory& trajectory) {
    double currentMaxVelocity = trajectory.points.front().twist.twist.linear.x;
    newVelocity = currentMaxVelocity + incrementFactor*velocityIncrement;
    if ( newVelocity < 0 ) {
        newVelocity = 0;
    }
    for ( auto trajectoryPoint = trajectory.points.begin();
    trajectoryPoint != trajectory.points.end(); ++trajectoryPoint ) {
        trajectoryPoint->twist.twist.linear.x = newVelocity;
    }
}
