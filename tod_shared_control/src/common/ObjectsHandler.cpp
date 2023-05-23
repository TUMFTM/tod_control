// Copyright 2020 Andreas Schimpe
#include "ObjectsHandler.h"

namespace tod_shared_control {

ObjectsHandler::ObjectsHandler(const std::string &controllerFrame)
    : _controllerFrame(controllerFrame),
    _transformListener{tf2_ros::TransformListener(_tfBuffer)} {
}

void ObjectsHandler::set_data(const tod_msgs::ObjectListConstPtr &msg) {
    if (!_printedInfo) {
        ROS_DEBUG("%s: First Object List Msg in %s!",
                  ros::this_node::getName().c_str(), msg->header.frame_id.c_str());
        _printedInfo = true;
    }
    _objects.clear();
    _objects = msg->objectList;
    _detectorFrame = msg->header.frame_id;
}

ObjectList ObjectsHandler::get_n_nearest_objects_from_xy(const int n, const double x, const double y) {
    ObjectList objsToReturn = get_transformed_objects(_detectorFrame, _controllerFrame);
    if (!_okay) {
        ROS_ERROR("%s: cannot get n nearest objects - status msg: %s - returning empty list",
                  ros::this_node::getName().c_str(), _statusMsg.c_str());
        return ObjectList();
    }
    std::sort(objsToReturn.begin(), objsToReturn.end(),
              [x, y](const tod_msgs::ObjectData &obj1, const tod_msgs::ObjectData& obj2) {
                  double d1 = std::sqrt(std::pow(obj1.distCenterX - x, 2) + std::pow(obj1.distCenterY - y, 2));
                  double d2 = std::sqrt(std::pow(obj2.distCenterX - x, 2) + std::pow(obj2.distCenterY - y, 2));
                  return (d1 < d2);
              });
    make_object_list_of_size_n(objsToReturn, n, x, y);
    return objsToReturn;
}

void ObjectsHandler::make_object_list_of_size_n(ObjectList &objects, const int n, const double x, const double y) {
    // fill or erase objects to get desired number of objects
    for (size_t i = objects.size(); i < n; ++i) {
        tod_msgs::ObjectData& farDummyObject = objects.emplace_back();
        farDummyObject.distCenterX = float(x) + 100.0f;
        farDummyObject.distCenterY = float(y) + 100.0f;
        farDummyObject.speedX = farDummyObject.speedY = 0.0f;
        farDummyObject.id = int32_t(1000 + i);
        farDummyObject.yawAngle = farDummyObject.dimY = 0.0f;
        farDummyObject.dimZ = farDummyObject.dimX = 0.0f;
        farDummyObject.accelerationX = farDummyObject.accelerationY = 0.0f;
        farDummyObject.classification = tod_msgs::ObjectData::CLASSIFICATION_UNKNOWN;
        objects.emplace_back(farDummyObject);
    }
    objects.erase(objects.begin() + n, objects.end());
}

ObjectList ObjectsHandler::get_transformed_objects(const std::string &srcFrame, const std::string &trgFrame) {
    ObjectList objects = _objects;
    geometry_msgs::TransformStamped tf;
    if (_tfBuffer.canTransform(trgFrame, srcFrame, ros::Time::now())) {
        tf = _tfBuffer.lookupTransform(trgFrame, srcFrame, ros::Time::now());
    } else {
        _statusMsg = std::string("cannot get transform from " + srcFrame + " to " + trgFrame);
        _okay = false;
        return objects;
    }
    tod_helper::ObjectList::transform(objects, tf);
    _statusMsg.clear();
    _okay = true;
    return objects;
}

}; // namespace tod_shared_control
