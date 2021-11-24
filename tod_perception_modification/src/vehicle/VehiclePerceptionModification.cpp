// Copyright 2021 Feiler

#include "VehiclePerceptionModification.h"
#include <utility>
#include "Transform.h"
#include "glm/glm.hpp"
#include "ObjectList.h"

VehiclePerceptionModification::VehiclePerceptionModification() :
tfListener(tfBuffer) {
    initGridMapWithMarkedArea("ftm");
    subscribers.push_back(nodeHandle.subscribe<grid_map_msgs::GridMapConstPtr>(
        "input_grid_map", 5, &VehiclePerceptionModification::callbackGridMap, this));
    publishers.insert(std::pair<std::string, ros::Publisher>(
        "GridMap", nodeHandle.advertise<grid_map_msgs::GridMap>(
        "output_grid_map", 5)));

    subscribers.push_back(nodeHandle.subscribe<tod_msgs::ObjectListConstPtr>(
        "input_object_list", 5, &VehiclePerceptionModification::callbackObjectList, this));
    publishers.insert(std::pair<std::string, ros::Publisher>(
        "ObjectList", nodeHandle.advertise<tod_msgs::ObjectList>(
        "output_object_list", 5)));

    subscribers.push_back(nodeHandle.subscribe<>(
        "apply_modification", 5, &VehiclePerceptionModification::callbackApplyModification, this));
    subscribers.push_back(nodeHandle.subscribe(
        "area", 5, &VehiclePerceptionModification::callbackArea, this));

    subscribers.push_back(nodeHandle.subscribe(
        "odometry", 5, &VehiclePerceptionModification::callbackOdometry, this));
}


void VehiclePerceptionModification::callbackGridMap(const grid_map_msgs::GridMapConstPtr msg) {
    grid_map::GridMap receivedGridMap;
    grid_map::GridMapRosConverter::fromMessage(*msg, receivedGridMap);
    if ( applyModification && !area.points.empty() ) {
        deleteAffectedCells(receivedGridMap);
    }
    grid_map_msgs::GridMap modifiedGridMap;
    grid_map::GridMapRosConverter::toMessage(receivedGridMap, modifiedGridMap);
    publishers["GridMap"].publish(modifiedGridMap);
}

void VehiclePerceptionModification::deleteAffectedCells(grid_map::GridMap& receivedGridMap) {
    grid_map::Polygon areaAsPolygon = createPolygonFromArea();
    setCellsInAreaToZero(receivedGridMap, areaAsPolygon);
}

grid_map::Polygon VehiclePerceptionModification::createPolygonFromArea() {
    grid_map::Polygon areaAsPolygon;
    for ( auto vertex : area.points ) {
        areaAsPolygon.addVertex(grid_map::Position(vertex.x, vertex.y));
    }
    return areaAsPolygon;
}

void VehiclePerceptionModification::setCellsInAreaToZero(
grid_map::GridMap& receivedGridMap, const grid_map::Polygon& areaAsPolygon) {
    grid_map::PolygonIterator polygonIterator(receivedGridMap, areaAsPolygon);
    for ( polygonIterator; !polygonIterator.isPastEnd(); ++polygonIterator ) {
        receivedGridMap.at(receivedGridMap.getLayers().front(), *polygonIterator) = 0.0;
    }
}

void VehiclePerceptionModification::callbackObjectList(const tod_msgs::ObjectListConstPtr msg) {
    tod_msgs::ObjectList modifiedObjectList = *msg;
    if ( applyModification && !area.points.empty() ) {
        deleteAffectedObjects(modifiedObjectList);
    }
    publishers["ObjectList"].publish(modifiedObjectList);
}

void VehiclePerceptionModification::deleteAffectedObjects(
tod_msgs::ObjectList& modifiedObjectList) {
    geometry_msgs::TransformStamped transformObjectsIntoAreaFrameID;
    if ( !TodHelper::getTransformToTargetFromSource(area.header.frame_id,
    modifiedObjectList.header.frame_id, transformObjectsIntoAreaFrameID, tfBuffer) ) {
        return;
    }
    std::vector<std::vector<glm::vec3>> objectListMeshOriginalFrame =
        TodHelper::ObjectList::createVectorOfObjectMeshes(modifiedObjectList);
    std::vector<std::vector<glm::vec3>> objectListMeshInAreaFrameID =
        TodHelper::ObjectList::transformVectorOfObjectMeshes(
        objectListMeshOriginalFrame, transformObjectsIntoAreaFrameID);
    nav_msgs::Odometry odomBehind = convertToPositionBehind(odometry, 5.0);
    TodHelper::ObjectList::resetAndMoveCurrentMarkedArea(gridMapWithMarkedArea,
                                    odomBehind);
    TodHelper::ObjectList::fillGridWithClicks(gridMapWithMarkedArea, area);
    std::vector<int> indicesToBeDeleted;
    for ( int index = 0; index != modifiedObjectList.objectList.size(); ++index ) {
        grid_map::Polygon objectAsPolygon = TodHelper::ObjectList::
            createFloorPolygonFromVectors(objectListMeshInAreaFrameID.at(index));
        if ( TodHelper::ObjectList::objectOverlapsWithArea(
        gridMapWithMarkedArea, objectAsPolygon) ) {
            indicesToBeDeleted.push_back(index);
        }
    }
    if ( !indicesToBeDeleted.empty() ) {
        TodHelper::ObjectList::removeRespectiveObjects(modifiedObjectList,
        indicesToBeDeleted);
    }
}

nav_msgs::Odometry VehiclePerceptionModification::convertToPositionBehind(
const nav_msgs::Odometry& odometry, const double& metersBehind) {
    double headingSelf{0.0};
    double factor{ 1.0 };
    if ( odometry.pose.pose.orientation.z > 0.0 ) { factor = 1.0; }
    else { factor = -1.0; }
    headingSelf = factor*2*std::acos(odometry.pose.pose.orientation.w);
    double xMapBehindVehicle = std::cos(headingSelf) * metersBehind;
    double yMapBehindVehicle = std::sin(headingSelf) * metersBehind;
    nav_msgs::Odometry odomBehind;
    odomBehind = odometry;
    odomBehind.pose.pose.position.x = odometry.pose.pose.position.x +
                                        xMapBehindVehicle;
    odomBehind.pose.pose.position.y = odometry.pose.pose.position.y +
                                        yMapBehindVehicle;
    return odomBehind;
}

void VehiclePerceptionModification::callbackApplyModification(const std_msgs::Bool& msg) {
    applyModification = msg.data;
}

void VehiclePerceptionModification::callbackArea(const sensor_msgs::PointCloud& msg) {
    area = msg;
}

void VehiclePerceptionModification::callbackOdometry(const nav_msgs::Odometry& msg) {
    odometry = msg;
}

void VehiclePerceptionModification::initGridMapWithMarkedArea(const std::string& frame_id) {
    gridMapWithMarkedArea.add("occupancyProbability");
    gridMapWithMarkedArea.setFrameId(frame_id);
    gridMapWithMarkedArea.setGeometry(grid_map::Length(20, 20), 0.1);
}

