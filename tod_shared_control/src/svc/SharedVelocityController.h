// Copyright 2021 Andreas Schimpe
#include <ros/ros.h>
#include <memory>
#include <tod_core/VehicleParameters.h>
#include <tod_helper/vehicle/Model.h>
#include <tod_msgs/ColoredPolygon.h>
#include <tod_shared_control/SvcLog.h>
#include <visualization_msgs/MarkerArray.h>

#include "ControllerBase.h"
#include "ObjectsHandler.h"
#include "TrajectoryTree.h"
#include "VelocityOptimizer.h"

namespace tod_shared_control {

class SharedVelocityController : public ControllerBase {
public:
    SharedVelocityController(ros::NodeHandle &nodeHandle);
    ~SharedVelocityController() override { }
    void run();

private:
    ros::Publisher _pubAvoidedObstacles, _pubPredictedPolygon, _pubTrajectoryTree, _pubLog;
    bool _logTrajectoriyTreeTrajectories{false};

    std::string _planningFrame;
    tod_shared_control::MultiObjectListHandler _objectsHandler;

    std::unique_ptr<tod_core::VehicleParameters> _coreVehParams;
    VehicleParams _treeVehParams;
    TreeParams _treeParams;
    std::unique_ptr<TrajectoryTreeCreator> _trajectoryPlanner{nullptr};
    std::unique_ptr<VelocityOptimizer> _velocityOptimizer{nullptr};

    tod_msgs::PrimaryControlCmd compute_control_command(std::vector<tod_msgs::ObjectData> &avoidedObjs, tod_shared_control::SvcLog &log);
    void publish_avoided_objects(std::vector<tod_msgs::ObjectData> &avoidedObjects);
    void publish_predicted_polygon();
    void publish_and_log_trajectory_tree(tod_shared_control::SvcLog &log);
    void publish_log(tod_shared_control::SvcLog &log);
};

} // namespace tod_shared_control
