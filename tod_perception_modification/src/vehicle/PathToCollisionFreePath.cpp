// Copyright 2021 Feiler

#include "PathToCollisionFreePath.h"
#include <ros/console.h>
#include <chrono>
#include "tod_helper/trajectory/BrakeIntoStillStand.h"

PathToCollisionFreePath::PathToCollisionFreePath() :
tfListener(tfBuffer) {
    subscribers.push_back(nodeHandle.subscribe<tod_msgs::TrajectoryConstPtr>(
        "input_trajectory", 5, &PathToCollisionFreePath::callbackInputTrajectory, this));
    subscribers.push_back(nodeHandle.subscribe("input_grid_map", 5,
                    &PathToCollisionFreePath::callbackInputGridMap, this));
    subscribers.push_back(nodeHandle.subscribe("input_object_list", 5,
                    &PathToCollisionFreePath::callbackInputObjectList, this));
    pubCollisionFreeTrajectory = nodeHandle.advertise<tod_msgs::Trajectory>(
        "output_trajectory", 2);
    std::string debug;
    if ( nodeHandle.getParam(ros::this_node::getName() + "/debug", debug) ) {
        if ( debug == "true" ) {
            turnDebuggingOn();
        }
    }
    loadParamsFromParamServer();
}

void PathToCollisionFreePath::callbackInputTrajectory(const tod_msgs::TrajectoryConstPtr msg) {
    if ( !inputObjectList.header.frame_id.empty() ) {
        addInputObjectListToInputGridMap();
    }
    tod_msgs::Trajectory outTrajectory = *msg;
    int index { 0 };
    bool trajectoryCollides { false };
    grid_map::Index gridMapPositionIndex;
    if ( trajectoryCollidesWithGridMapData(inputGridMap, outTrajectory, index,
    gridMapPositionIndex) ) {
        trajectoryCollides = true;
        tellTheUserEveryXSecondThatSomethingWouldCollide(5);
    }
    if ( !trajectoryCollides ) { // nothing to change
        pubCollisionFreeTrajectory.publish(outTrajectory);
        return;
    }
    tellTheUserIfChildFrameIdIsNotCorrect(outTrajectory);
    int indexOfDesiredTrajectoryStopPoint = getIndexOfDesiredRearAxleStopPoint(
        outTrajectory, index, gridMapPositionIndex);
    modifyTrajectoryToBrakeAppropriately(outTrajectory, indexOfDesiredTrajectoryStopPoint);
    pubCollisionFreeTrajectory.publish(outTrajectory);
}

void PathToCollisionFreePath::addInputObjectListToInputGridMap() {
    geometry_msgs::TransformStamped transform;
    if ( !getTransformToTargetFromSource(inputGridMap.getFrameId(),
            inputObjectList.header.frame_id, transform) ) {
        return; // can not add without transform
    }
    std::vector<std::vector<glm::vec3>> objectListAsGLMvecLocally =
        TodHelper::ObjectList::createVectorOfObjectMeshes(inputObjectList);
    std::vector<std::vector<glm::vec3>> objectListAsGLMvec =
        TodHelper::ObjectList::transformVectorOfObjectMeshes(
        objectListAsGLMvecLocally, transform);
    for ( int index = 0; index != inputObjectList.objectList.size(); ++index ) {
        grid_map::Polygon objectAsPolygon = TodHelper::ObjectList::
            createFloorPolygonFromVectors(objectListAsGLMvec.at(index));
        fillGridWithFloorPolygon(objectAsPolygon);
    }
}

bool PathToCollisionFreePath::getTransformToTargetFromSource(const std::string& targetFrame,
const std::string& sourceFrame,
geometry_msgs::TransformStamped& transform) {
    bool gotTransform { false };
    try {
        transform = tfBuffer.lookupTransform(
            targetFrame, sourceFrame, ros::Time(0));
        gotTransform = true;
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s at %s", ex.what(), ros::this_node::getName().c_str());
    }
    return gotTransform;
}

void PathToCollisionFreePath::fillGridWithFloorPolygon(const grid_map::Polygon& objectAsPolygon) {
    grid_map::PolygonIterator polygonIterator(inputGridMap, objectAsPolygon);
    for ( polygonIterator; !polygonIterator.isPastEnd(); ++polygonIterator ) {
        inputGridMap.at(inputGridMap.getLayers().front(), *polygonIterator) = 1.0;
    }
}

bool PathToCollisionFreePath::trajectoryCollidesWithGridMapData(
const grid_map::GridMap& gridMap,
const tod_msgs::Trajectory& trajectory, int& trajectoryIndex,
grid_map::Index& gridMapPositionIndex) {
    bool trajectoryCollides { false };
    for ( int iterator = 0; iterator != trajectory.points.size(); ++iterator ) {
        if ( gridMap.getLayers().size() == 0 ) {
            break; // grid map not initialized yet
        }
        grid_map::Position center;
        center.x() = trajectory.points.at(iterator).pose.pose.position.x;
        center.y() = trajectory.points.at(iterator).pose.pose.position.y;
        for (grid_map::CircleIterator circleIterator(gridMap, center, radiusBuffer + vehicleWidth/2);
        !circleIterator.isPastEnd(); ++circleIterator) {
            double cellValue = gridMap.at(gridMap.getLayers().front(), *circleIterator);
            if ( cellValue != free && !std::isnan(cellValue) ) {
                trajectoryCollides = true;
                trajectoryIndex = iterator;
                gridMapPositionIndex = *circleIterator;
                break;
            }
        }
        if ( trajectoryCollides ) {
            break;
        }
    }
    return trajectoryCollides;
}

void PathToCollisionFreePath::callbackInputGridMap(const grid_map_msgs::GridMap& msg) {
    if (!grid_map::GridMapRosConverter::fromMessage(msg, inputGridMap)) {
        ROS_WARN("could not convert grid map from ros msg successfully at %s",
            ros::this_node::getName().c_str());
    }
}

void PathToCollisionFreePath::callbackInputObjectList(const tod_msgs::ObjectList& msg) {
    inputObjectList = msg;
}

void PathToCollisionFreePath::turnDebuggingOn() {
    if ( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
    ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }
}

void PathToCollisionFreePath::loadParamsFromParamServer() {
    loadParam("width_edge_to_edge", vehicleWidth, 2.0);
    loadParam("FREE", free, 0.0);
    loadParam("distance_front_bumper", distanceFrontBumper, 2.408);
    loadParam("distance_rear_axle", distanceRearAxle, 1.556);
    loadParam("buffer_distance_in_meter", bufferDistanceInM, 1.5);
    loadParam("brake_acceleration_in_m_s_2", brakeAcceleration, -2.0);
    loadParam("radius_buffer", radiusBuffer, 1.0);
}

void PathToCollisionFreePath::printStandardRosWarn(const std::string& paramName) {
    ROS_WARN("could not find %s on param server at %s. Using default value.",
        paramName.c_str(), ros::this_node::getName().c_str());
}

void PathToCollisionFreePath::tellTheUserIfChildFrameIdIsNotCorrect(
const tod_msgs::Trajectory& outTrajectory) {
    if ( outTrajectory.child_frame_id != "rear_axle_footprint" ) {
        ROS_ERROR("trajectory has the wrong child_frame_id (%s).",
        outTrajectory.child_frame_id.c_str());
    }
}

int PathToCollisionFreePath::getIndexOfDesiredRearAxleStopPoint(const tod_msgs::Trajectory& outTrajectory,
        const int index, grid_map::Index& gridMapPositionIndex) {
    int indexOfDesiredTrajectoryStopPoint { 0 };
    grid_map::Position obstaclePosition;
    if ( !inputGridMap.getPosition(gridMapPositionIndex, obstaclePosition) ) {
        ROS_ERROR("could not get obstacle position from grid map. %s",
        ros::this_node::getName().c_str());
    }
    double desiredDistanceRearAxleToObstacle =
        distanceFrontBumper + distanceRearAxle + bufferDistanceInM;
    for ( int iterator = index; iterator != 0; --iterator ) {
        double xTraj = outTrajectory.points.at(iterator).pose.pose.position.x;
        double yTraj = outTrajectory.points.at(iterator).pose.pose.position.y;
        double xObst = obstaclePosition.x();
        double yObst = obstaclePosition.y();
        double distance = std::sqrt(std::pow(xTraj-xObst, 2) + std::pow(yTraj-yObst, 2));
        if ( distance > desiredDistanceRearAxleToObstacle ) {
            indexOfDesiredTrajectoryStopPoint = iterator;
            break;
        }
    }
    return indexOfDesiredTrajectoryStopPoint;
}

void PathToCollisionFreePath::modifyTrajectoryToBrakeAppropriately(
tod_msgs::Trajectory& outTrajectory, const int indexOfDesiredTrajectoryStopPoint) {
    double goalVelocity = outTrajectory.points.front().twist.twist.linear.x;
    tod_helper::Trajectory::brakeIntoStillStand(outTrajectory,
    indexOfDesiredTrajectoryStopPoint, goalVelocity, brakeAcceleration);
}

void PathToCollisionFreePath::tellTheUserEveryXSecondThatSomethingWouldCollide(
const int seconds) {
    static bool firstTime { true };
    static ros::Time timeStampOfPrint;
    if ( firstTime ) {
        ROS_ERROR("Something would collide at %s", ros::this_node::getName().c_str());
        firstTime = false;
        timeStampOfPrint = ros::Time::now();
    }
    if ( !firstTime && ((ros::Time::now().sec-timeStampOfPrint.sec) > seconds) ) {
        // print again
        firstTime = true;
    }
}
