// Copyright 2020 Andreas Schimpe
#pragma once
#include <ros/ros.h>
#include <math.h>
#include <chrono>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <tod_msgs/ColoredPolygon.h>
#include <tod_helper/vehicle/Model.h>
#include <tod_helper/geometry/Helpers.h>
#include <tod_shared_control/MpcLog.h>
#include <tod_helper/colored_polygon/Helpers.h>

#include "ControllerBase.h"
#include "ObjectsHandler.h"
#include "MpcSolver.h"

namespace tod_shared_control {

class SharedController : public ControllerBase {
public:
    explicit SharedController(ros::NodeHandle& nodeHandle);
    ~SharedController() override { }
    void run();

private:
    ros::Publisher _pubAvoidedObstacles, _pubMpcLog, _pubPredictedPolygon;

    std::shared_ptr<tod_shared_control::MpcSolver::Parameters> _ctrlerParams{nullptr};

    // feedback
    std::string _planningFrame;
    tod_shared_control::MultiObjectListHandler _objectsHandler;
    std::unique_ptr<tod_shared_control::MpcSolver> _solver{nullptr};

    tod_msgs::PrimaryControlCmd execute_control(const ros::Rate &controllerRate);
    void publish_avoided_obstacles(const std::vector<tod_msgs::ObjectData> objects);
    void publish_predicted_polygon(tod_msgs::ColoredPolygon &predictedPolygon);
    void publish_log(tod_shared_control::MpcLog &logMsg);

    tod_shared_control::BicycleState get_reference_state();
    tod_shared_control::BicycleState get_initial_state();
};

}; // namespace tod_shared_control
