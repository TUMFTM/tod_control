// Copyright 2021 Feiler

#include "FakeCellsPublisher.h"
#include "geometry_msgs/TransformStamped.h"

FakeCellsPublisher::FakeCellsPublisher() : itsTf2Listener(itsTf2Buffer) {
    currentMode = Mode::APPEND;
    std::string topicName{"input_grid_map"};
    subscribers.push_back(nodeHandle.subscribe<grid_map_msgs::GridMapConstPtr>(
        topicName, 5, &FakeCellsPublisher::callbackGridMap, this));
    subscribers.push_back(nodeHandle.subscribe("odometry", 1,
                    &FakeCellsPublisher::processArrivedOdomFrame, this));
    pubGridMap = nodeHandle.advertise<grid_map_msgs::GridMap>(
        "new_grid_map", 2);
    timer = std::make_unique<Timer>(4);
}

void FakeCellsPublisher::initFromParamServer(const std::string& nodeName) {
    int maximumElements = 10;
    for ( int reflectionNumber = 0; reflectionNumber < maximumElements; ++reflectionNumber ) {
        lookUpAndAddReflection(reflectionNumber, nodeName);
    }
}

void FakeCellsPublisher::lookUpAndAddReflection(const int reflectionNumber,
const std::string& nodeName) {
    bool gotfakeReflection = true;
    std::vector<double> coordinates_x_y_z;
    if (!nodeHandle.getParam(nodeName + "/LidarPoint" + std::to_string(reflectionNumber)
            + "/coordinates_x_y_z", coordinates_x_y_z)) {
        gotfakeReflection = false;
    } else {
        grid_map::Position tmpPosition;
        tmpPosition.x() = coordinates_x_y_z.at(0);
        tmpPosition.y() = coordinates_x_y_z.at(1);
        reflections.push_back(tmpPosition);
    }
    std::string frame_id;
    if (!nodeHandle.getParam(nodeName + "/LidarPoint" + std::to_string(reflectionNumber)
            + "/frame_id", frame_id)) {
        gotfakeReflection = false;
    } else {
        reflectionsFrameId = frame_id;
    }
}

void FakeCellsPublisher::callbackGridMap(const grid_map_msgs::GridMapConstPtr msg) {
    timer->reset();
    if (!grid_map::GridMapRosConverter::fromMessage(*msg, gridMap)) {
        ROS_WARN("could not convert grid map from ros msg successfully at %s",
            ros::this_node::getName().c_str());
    }
    if ( !reflections.empty() ) {
        appendFakeReflectionsToGridMap();
    }
    publish();
}

void FakeCellsPublisher::processArrivedOdomFrame(const nav_msgs::Odometry& msg) {
    if ( isCreateMode() && !gridMap.getLayers().empty() ) {
        moveGridMapAccordingToOdometry(msg);
    }
}

void FakeCellsPublisher::moveGridMapAccordingToOdometry(const nav_msgs::Odometry& msg) {
    geometry_msgs::TransformStamped transformStamped;
    if ( !itsTf2Buffer.canTransform(gridMapFrameId, msg.header.frame_id, ros::Time(0)) ) {
        return;
    }
    try {
        transformStamped = itsTf2Buffer.lookupTransform(gridMapFrameId,
                                msg.header.frame_id, ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
    }
    geometry_msgs::Point outPoint;
    geometry_msgs::Point inPoint = msg.pose.pose.position;
    tf2::doTransform(inPoint, outPoint, transformStamped);

    double headingSelf{0.0};
    double factor{ 1.0 };
    if ( msg.pose.pose.orientation.z > 0.0 ) { factor = 1.0;
    } else { factor = -1.0; }
    headingSelf = factor*2*std::acos(msg.pose.pose.orientation.w);
    double xMapBehindVehicle = std::cos(headingSelf) * mapBehindVehicleInM;
    double yMapBehindVehicle = std::sin(headingSelf) * mapBehindVehicleInM;
    grid_map::Position currentVehiclePosition;
    currentVehiclePosition.x() = outPoint.x;
    currentVehiclePosition.y() = outPoint.y;
    gridMap.move(grid_map::Position(currentVehiclePosition.x() +
        xMapBehindVehicle, currentVehiclePosition.y()+yMapBehindVehicle));
}

bool FakeCellsPublisher::isCreateMode() {
    bool isInCreateMode {false};
    if ( currentMode == Mode::CREATE ) {
        isInCreateMode = true;
    } else if ( timer->expired() ) {
        currentMode = Mode::CREATE;
        isInCreateMode = true;
    }
    return isInCreateMode;
}

void FakeCellsPublisher::create() {
    static bool firstTime { true };
    if ( firstTime ) {
        firstTime = false;
        loadParamsFromParamServer();
        initGridMap();
    }
    if ( !reflections.empty() ) {
        appendFakeReflectionsToGridMap();
    }
}

void FakeCellsPublisher::initGridMap() {
    gridMap.add("occupancyProbability");
    gridMap.setFrameId(gridMapFrameId);
    gridMap.setGeometry(grid_map::Length(gridMapWidthInM, gridMapHeightInM),
                        gridMapCellSizeInM);
    ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
                gridMap.getLength().x(), gridMap.getLength().y(),
                gridMap.getSize()(0), gridMap.getSize()(1));
}

void FakeCellsPublisher::appendFakeReflectionsToGridMap() {
    if ( reflectionsFrameId != gridMap.getFrameId() ) {
        ROS_ERROR("provide reflections in frame id of gridmap.");
        ROS_ERROR("gridmap frame id is %s. reflections frame id is %s.",
            gridMap.getFrameId().c_str(), reflectionsFrameId.c_str());
        return;
    }
    for ( auto reflection : reflections ) {
        if ( std::isnan(reflection.x()) || std::isnan(reflection.y()) ) {
            continue;
        }
        if ( gridMap.isInside(reflection) ) {
            double occupied { 1.0 };
            gridMap.atPosition(gridMap.getLayers().front(), reflection) = occupied;
        }
    }
}

void FakeCellsPublisher::publish() {
    grid_map_msgs::GridMap tmpToBePublished;
    grid_map::GridMapRosConverter::toMessage(gridMap, tmpToBePublished);
    pubGridMap.publish(tmpToBePublished);
}

void FakeCellsPublisher::calcAndPrintBitRate(const grid_map_msgs::GridMap& map,
int nodeFrequency) {
    ros::SerializedMessage rosSer =
        ros::serialization::serializeMessage<grid_map_msgs::GridMap>(map);
    static double byteCount = 0;
    byteCount += rosSer.num_bytes;
    static int counter = 0;
    counter += 1;
    if ( counter == 9 ) {
        ROS_ERROR("MBit/s = %f", byteCount * 8 / 1000000);
        byteCount = 0;
        counter = 0;
    }
}

void FakeCellsPublisher::loadParamsFromParamServer() {
    loadParam("GRID_MAP_WIDTH_IN_M", gridMapWidthInM, 20.0);
    loadParam("GRID_MAP_HEIGHT_IN_M", gridMapHeightInM, 20.0);
    loadParam("GRID_CELL_SIZE_IN_M", gridMapCellSizeInM, 0.2);
    loadParam("MAP_BEHIND_VEHICLE_IN_M", mapBehindVehicleInM, 7.0);
    loadParam("LIDAR_HEIGHT", lidarHeight, 0.172);
    loadParam("OCCUPIED", occupied, 1.0);
    loadParam("FREE", free, 0.0);
}

void FakeCellsPublisher::printStandardRosWarn(const std::string& paramName) {
    ROS_WARN("could not find %s on param server at %s. Using default value.",
        paramName.c_str(), ros::this_node::getName().c_str());
}

