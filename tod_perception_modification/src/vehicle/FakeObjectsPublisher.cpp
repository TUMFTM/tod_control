// Copyright 2021 Feiler

#include "FakeObjectsPublisher.h"

FakeObjectsPublisher::FakeObjectsPublisher() : tfListener(tfBuffer) {
    currentMode = Mode::APPEND;
    std::string topicName{"input_object_list"};
    subscribers.push_back(nodeHandle.subscribe<tod_msgs::ObjectListConstPtr>(
        topicName, 5, &FakeObjectsPublisher::callbackObjectList, this));
    pubObjectList = nodeHandle.advertise<tod_msgs::ObjectList>(
        "new_object_list", 2);
    timer = std::make_unique<Timer>(4);
}

void FakeObjectsPublisher::initFromParamServer(const std::string& nodeName) {
    int maximumElements = 10;
    for ( int objectNumber = 0; objectNumber < maximumElements; ++objectNumber ) {
        lookUpAndAddObject(objectNumber, nodeName);
    }
    loadDesiredObjectsFrameId(nodeName);
}

void FakeObjectsPublisher::lookUpAndAddObject(const int objectNumber,
const std::string& nodeName) {
    bool gotfakeObject = true;
    std::vector<double> dimension_length_width_height;
    if (!nodeHandle.getParam(nodeName + "/Object" + std::to_string(objectNumber)
            + "/dimension_length_width_height", dimension_length_width_height)) {
        gotfakeObject = false;
    }
    std::vector<double> distance_x_y;
    if (!nodeHandle.getParam(nodeName + "/Object" + std::to_string(objectNumber)
            + "/distance_x_y", distance_x_y)) {
        gotfakeObject = false;
    }
    std::string frame_id;
    if (!nodeHandle.getParam(nodeName + "/Object" + std::to_string(objectNumber)
            + "/frame_id", frame_id)) {
        gotfakeObject = false;
    } else {
        fakeObjects.header.frame_id = frame_id;
    }
    int objectRefPoint;
    if (!nodeHandle.getParam(nodeName + "/Object" + std::to_string(objectNumber)
            + "/objectRefPoint", objectRefPoint)) {
        gotfakeObject = false;
    }
    double yaw_angle;
    nodeHandle.getParam(nodeName + "/Object" + std::to_string(objectNumber)
            + "/yaw_angle", yaw_angle);
    if ( gotfakeObject ) {
        tod_msgs::ObjectData fakeObject;
        fakeObject.dimX = dimension_length_width_height.at(0);
        fakeObject.dimY = dimension_length_width_height.at(1);
        fakeObject.dimZ = dimension_length_width_height.at(2);
        fakeObject.distCenterX = distance_x_y.at(0);
        fakeObject.distCenterY = distance_x_y.at(1);
        fakeObject.yawAngle = yaw_angle;
        fakeObjects.objectList.push_back(fakeObject);
    }
}

void FakeObjectsPublisher::loadDesiredObjectsFrameId(const std::string& nodeName) {
    std::string paramName = "/desiredObjectsFrameId";
    std::string defaultValue = "LidarFront";
    if (!nodeHandle.getParam(nodeName + paramName, desiredObjectsFrameId)) {
        ROS_WARN("Could not load %s from paramServer at %s. Taking default value %s",
            paramName.c_str(), nodeName.c_str(), defaultValue.c_str());
        desiredObjectsFrameId = defaultValue;
    }
}

void FakeObjectsPublisher::callbackObjectList(const tod_msgs::ObjectListConstPtr msg) {
    timer->reset();
    objectListToBePublished = *msg;
    if ( !objectListToBePublished.objectList.empty() ) {
        appendFakeObjectsToList();
    }
    publish();
}

bool FakeObjectsPublisher::isCreateMode() {
    bool isInCreateMode {false};
    if ( currentMode == Mode::CREATE ) {
        isInCreateMode = true;
    } else if ( timer->expired() ) {
        currentMode = Mode::CREATE;
        isInCreateMode = true;
    }
    return isInCreateMode;
}

void FakeObjectsPublisher::create() {
    static bool firstTime { true };
    if ( firstTime ) {
        firstTime = false;
        objectListToBePublished.header.frame_id = desiredObjectsFrameId;
    }
    objectListToBePublished.objectList.clear();
    if ( !fakeObjects.objectList.empty() ) {
        appendFakeObjectsToList();
    }
}

void FakeObjectsPublisher::appendFakeObjectsToList() {
    geometry_msgs::TransformStamped transformFromTo;
    bool gotTransform { false };
    try {
        transformFromTo = tfBuffer.lookupTransform(
            objectListToBePublished.header.frame_id,
            fakeObjects.header.frame_id, ros::Time(0));
        gotTransform = true;
    } catch (tf2::TransformException &ex) {
        ROS_WARN_ONCE("%s: failed to look up tf with %s", ros::this_node::getName().c_str(), ex.what());
        ros::Duration(1.0).sleep();
    }
    if (gotTransform) {
        for ( int objectNumber = 0; objectNumber != fakeObjects.objectList.size();
        ++objectNumber ) {
            geometry_msgs::PoseStamped outPoint, inPoint;
            inPoint.pose.position.x = fakeObjects.objectList.at(objectNumber).distCenterX;
            inPoint.pose.position.y = fakeObjects.objectList.at(objectNumber).distCenterY;
            tf2::Quaternion quaternion;
            quaternion.setRPY(0.0, 0.0, fakeObjects.objectList.at(objectNumber).yawAngle);
            inPoint.pose.orientation.x = (double)quaternion.x();
            inPoint.pose.orientation.y = (double)quaternion.y();
            inPoint.pose.orientation.z = (double)quaternion.z();
            inPoint.pose.orientation.w = (double)quaternion.w();
            tf2::doTransform(inPoint, outPoint, transformFromTo);
            tod_msgs::ObjectData tmpFakeObject;
            tmpFakeObject = fakeObjects.objectList.at(objectNumber);
            tmpFakeObject.distCenterX = outPoint.pose.position.x;
            tmpFakeObject.distCenterY = outPoint.pose.position.y;
            tf2::Quaternion tmpQuat(outPoint.pose.orientation.x, outPoint.pose.orientation.y,
                outPoint.pose.orientation.z, outPoint.pose.orientation.w);
            tf2::Matrix3x3 tmpMatrix(tmpQuat);
            tf2Scalar tmp, tmp2, yawAngle;
            tmpMatrix.getRPY(tmp, tmp2, yawAngle);
            tmpFakeObject.yawAngle = yawAngle;
            if ( tmpFakeObject.distCenterX > -1.5f ) { // only objects in front of lidar
                objectListToBePublished.objectList.push_back(tmpFakeObject);
            }
        }
        if ( objectListToBePublished.header.frame_id.empty() ) {
            objectListToBePublished.header.frame_id = fakeObjects.header.frame_id;
        }
    } else {
        ROS_ERROR_ONCE("%s: did not get transform for fake object",
                        ros::this_node::getName().c_str());
    }
}

void FakeObjectsPublisher::publish() {
    objectListToBePublished.header.stamp = ros::Time::now();
    pubObjectList.publish(objectListToBePublished);
}

