// Copyright 2020 Andreas Schimpe
#pragma once
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tod_msgs/ObjectList.h>
#include <tod_msgs/VehicleEnums.h>
#include <tod_helper/object_list/Helpers.h>
#include <string>
#include <algorithm>
#include <vector>
#include <memory>

namespace tod_shared_control {

typedef std::vector<tod_msgs::ObjectData> ObjectList;

class ObjectsHandler {
public:
    explicit ObjectsHandler(const std::string &targetFrame);
    ~ObjectsHandler() {}
    void set_data(const tod_msgs::ObjectListConstPtr &msg);
    void clear_data() { _objects.clear(); _okay = true; _statusMsg.clear(); }
    ObjectList get_n_nearest_objects_from_xy(const int n, const double x, const double y);

private:
    std::string _controllerFrame, _detectorFrame{""};
    tf2_ros::Buffer _tfBuffer;
    tf2_ros::TransformListener _transformListener;
    ObjectList _objects;
    bool _printedInfo{false}, _okay{true};
    std::string _statusMsg{""};
    ObjectList get_transformed_objects(const std::string &srcFrame, const std::string &trgFrame);
    static void make_object_list_of_size_n(ObjectList &objects, const int n, const double x, const double y);
};

class MultiObjectListHandler {
public:
    MultiObjectListHandler(ros::NodeHandle &_nh, const std::string planningFrame) :
        _frontObjectsHandler{planningFrame},
        _rearObjectsHandler{planningFrame} {
        std::string gridMapObjectsTopic{"/Vehicle/Lidar/GridMap/object_list"};
        _subFront = _nh.subscribe(
            gridMapObjectsTopic, 1, &tod_shared_control::ObjectsHandler::set_data,
            &_frontObjectsHandler);
        std::string rearObjectsTopic{"/Vehicle/Lidar/LidarRear/object_list"};
        _subRear = _nh.subscribe(
            rearObjectsTopic, 1, &tod_shared_control::ObjectsHandler::set_data,
            &_rearObjectsHandler);
    }
    ~MultiObjectListHandler() { }
    void reset() { _frontObjectsHandler.clear_data(); _rearObjectsHandler.clear_data(); }

    ObjectList get_n_nearest_objects_from_xy(const int n, const double x, const double y, const int gear) {
        ObjectsHandler& objHandler = (gear == GEARPOSITION_REVERSE) ? _rearObjectsHandler : _frontObjectsHandler;
        return objHandler.get_n_nearest_objects_from_xy(n, x, y);
    }

private:
    ros::Subscriber _subFront, _subRear;
    tod_shared_control::ObjectsHandler _frontObjectsHandler, _rearObjectsHandler;
};

}; // namespace tod_shared_control
