// Copyright 2021 Andreas Schimpe
#include "SharedVelocityController.h"

namespace tod_shared_control {

SharedVelocityController::SharedVelocityController(ros::NodeHandle &nodeHandle) :
    ControllerBase{nodeHandle},
    _planningFrame{"base_footprint"},
    _objectsHandler{_nh, _planningFrame},
    _coreVehParams{std::make_unique<tod_core::VehicleParameters>(_nh)} {
    _pubAvoidedObstacles = _nh.advertise<tod_msgs::ObjectList>("avoided_obstacles", 1);
    _pubPredictedPolygon = _nh.advertise<tod_msgs::ColoredPolygon>("predicted_polygon", 1);
    _pubTrajectoryTree = _nh.advertise<visualization_msgs::MarkerArray>("trajectory_tree", 1);
    _pubLog = _nh.advertise<tod_shared_control::SvcLog>("svc_log", 1);

    _nh.getParam(_nn + "/log_trajectory_tree_trajectories", _logTrajectoriyTreeTrajectories);

    _treeVehParams.LengthFrontAxle =_coreVehParams->get_distance_front_axle();
    _treeVehParams.LengthRearAxle =_coreVehParams->get_distance_rear_axle();
    _treeVehParams.LengthFrontBumper =_coreVehParams->get_distance_front_bumper();
    _treeVehParams.LengthRearBumper =_coreVehParams->get_distance_rear_bumper();
    _treeVehParams.Width =_coreVehParams->get_width();
    _treeVehParams.MaxDelta =_coreVehParams->get_max_rwa_rad();
    _treeVehParams.MaxDDelta = deg2rad(60.0f);
    _treeParams.TperStep = 0.050f;
    _treeParams.NofSegments = 1;
    _treeParams.TperSegment = 2.0f;
    _treeParams.NoSteeringInFinalSegment = false;
    _treeParams.NofDDeltas = 17; // make uneven to include ddelta == 0.0
    _treeParams.TrajectoryTimeHorizon = _treeParams.TperSegment * _treeParams.NofSegments;
    _trajectoryPlanner = std::make_unique<TrajectoryTreeCreator>(_treeParams, _treeVehParams);

    VelocityOptimizer::Constraints constraints;
    constraints.velMax = 10.0;
    constraints.accMin = -3.5;
    constraints.accMax = 2.0;
    constraints.latAccMax = std::sqrt(std::pow(9.81/2.0, 2.0) - std::pow(constraints.accMin, 2.0));
    constraints.jerkMax = 15.0;
    _velocityOptimizer = std::make_unique<VelocityOptimizer>(constraints);
}

void SharedVelocityController::run() {
    if (!_velocityOptimizer->ready()) {
        ROS_ERROR("%s: Velocity Optimizer not ready - terminating", _nn.c_str());
        return;
    }

    ros::Rate controllerRate(20);
    if (!_velocityOptimizer->cycle_time_consistent(controllerRate.expectedCycleTime().toSec())) {
        ROS_WARN("%s: ctrler cycle time (%.2fs) and mpc sample time (%.2fs) are inconsistent", _nn.c_str(),
                 controllerRate.expectedCycleTime().toSec(), _velocityOptimizer->get_sampling_time());
    }
    while (ros::ok()) {
        controllerRate.sleep();
        ros::spinOnce();
        if (!_fb.in_shared_control_and_teleoperation() || !_fb.feedback_complete()) {
            continue;
        }
        const auto t0 = ros::Time::now();
        std::vector<tod_msgs::ObjectData> avoidedObjs;
        tod_shared_control::SvcLog log;
        log.header.frame_id = _planningFrame;
        tod_msgs::PrimaryControlCmd ctrlCmd = compute_control_command(avoidedObjs, log);
        publish_ctrl_cmd(ctrlCmd);
        publish_avoided_objects(avoidedObjs);
        publish_predicted_polygon();
        publish_and_log_trajectory_tree(log);
        log.totalControllerTimeMs = (ros::Time::now() - t0).toSec() * 1000.0f;
        publish_log(log);
        check_cycle_time(controllerRate);
    }
}

tod_msgs::PrimaryControlCmd SharedVelocityController::compute_control_command(
    std::vector<tod_msgs::ObjectData> &avoidedObjs, tod_shared_control::SvcLog &log) {
    const auto t0 = ros::Time::now();

    // plan trajectory tree
    State currentState;
    currentState.X = 0.0f;
    currentState.Y = 0.0f;
    currentState.Theta = 0.0f;
    currentState.Delta = _coreVehParams->compute_rwa_from(_fb.vehicle_data()->steeringWheelAngle);
    currentState.Velocity = std::max(_fb.vehicle_data()->longitudinalSpeed, 1.0f);
    currentState.Progress = 0.0f;
    const float deceleration = -currentState.Velocity / _treeParams.TrajectoryTimeHorizon;
    _trajectoryPlanner->plan_trees_from(currentState, deceleration);
    const auto t1 = ros::Time::now();

    // check tree for collisions with obstacles
    avoidedObjs = _objectsHandler.get_n_nearest_objects_from_xy(5, currentState.X, currentState.Y, _fb.vehicle_data()->gearPosition);
    ObstacleList plannerObjs;
    for (const auto &obj : avoidedObjs) {
        auto& plannerObj = plannerObjs.emplace_back();
        plannerObj.CenterX = obj.distCenterX;
        plannerObj.CenterY = obj.distCenterY;
        plannerObj.DimX = obj.dimX;
        plannerObj.DimY = obj.dimY;
        plannerObj.Heading = obj.yawAngle;
    }
    VelocityOptimizer::Parameters parameters;
    parameters.safeProgress = _trajectoryPlanner->get_safe_progress_for(plannerObjs);
    _trajectoryPlanner->get_critical_profiles(parameters.curvatures, log.criticalSteeringAngleProfile);
    const auto t2 = ros::Time::now();

    // optimize velocity profile
    double currentVelocity = _fb.vehicle_data()->longitudinalSpeed;
    double currentProgress = double(currentState.Progress);
    double targetVelocity = _fb.primary_ctrl_cmd()->velocity;
    double jerkCmd, accelerationCmd, velocityCmd;
    bool solvable = _velocityOptimizer->solve(currentVelocity, targetVelocity, currentProgress, parameters,
                                              jerkCmd, accelerationCmd, velocityCmd);
    if (!solvable) ROS_WARN("%s: Velocity optimizer failed", _nn.c_str());
    const auto t3 = ros::Time::now();

    // write control command
    tod_msgs::PrimaryControlCmd ctrlCmd;
    ctrlCmd.steeringWheelAngle = _fb.primary_ctrl_cmd()->steeringWheelAngle;
    ctrlCmd.velocity = float(velocityCmd);
    ctrlCmd.acceleration = float(accelerationCmd);
    if (!solvable || parameters.safeProgress < 0.1f) {
        ctrlCmd.velocity = ctrlCmd.acceleration = 0.0;
    }

    // write log
    log.trajectoryPlannerTimeMs = (t1 - t0).toSec() * 1000.0f;
    log.collisionCheckerTimeMs = (t2 - t1).toSec() * 1000.0f;
    log.velocityOptimizerTimeMs = (t3 - t2).toSec() * 1000.0f;
    log.desiredAcceleration = _fb.primary_ctrl_cmd()->acceleration;
    log.desiredVelocity = _fb.primary_ctrl_cmd()->velocity;
    log.desiredRWA = _coreVehParams->compute_rwa_from(_fb.primary_ctrl_cmd()->steeringWheelAngle);
    log.velocityMax = _velocityOptimizer->get_constraints().velMax;
    log.accelerationMin = _velocityOptimizer->get_constraints().accMin;
    log.accelerationMax = _velocityOptimizer->get_constraints().accMax;
    log.lateralAccelerationMax = _velocityOptimizer->get_constraints().latAccMax;
    log.jerkMax = _velocityOptimizer->get_constraints().jerkMax;
    log.objectList = avoidedObjs;
    log.odometry = *_fb.odometry();
    log.safeProgress = parameters.safeProgress;
    log.criticalCurvatureProfile = parameters.curvatures;
    log.jerkCmd = jerkCmd;
    log.accelerationCmd = ctrlCmd.acceleration;
    log.velocityCmd = ctrlCmd.velocity;
    log.rwaCmd = _coreVehParams->compute_rwa_from(ctrlCmd.steeringWheelAngle);
    _velocityOptimizer->get_optimized_profiles(log.progressProfile, log.velocityProfile, log.accelerationProfile, log.jerkProfile);
    for (int i=0; i < log.velocityProfile.size(); ++i) {
        log.lateralAccelerationProfile.push_back(std::pow(log.velocityProfile.at(i), 2) * parameters.curvatures.at(i));
    }
    for (int i=0; i <= _velocityOptimizer->get_prediction_horizon(); ++i) {
        log.timeProfile.emplace_back(double(i) * _velocityOptimizer->get_sampling_time());
    }
    log.vehicleLengthFront = _coreVehParams->get_distance_front_axle();
    log.vehicleLengthRear = _coreVehParams->get_distance_rear_axle();
    log.vehicleLengthFrontBumper = _coreVehParams->get_distance_front_bumper();
    log.vehicleLengthRearBumper = _coreVehParams->get_distance_rear_bumper();
    log.vehicleWidth = _coreVehParams->get_width();

    return ctrlCmd;
}

void SharedVelocityController::publish_avoided_objects(std::vector<tod_msgs::ObjectData> &avoidedObjects) {
    tod_msgs::ObjectList msg;
    msg.header.stamp = ros::Time::now();
    msg.objectList = avoidedObjects;
    msg.header.frame_id = _planningFrame;
    _pubAvoidedObstacles.publish(msg);
}

void SharedVelocityController::publish_predicted_polygon() {
    static std_msgs::ColorRGBA RED; RED.a = RED.r = 1.0;
    static std_msgs::ColorRGBA GREEN; GREEN.a = GREEN.g = 1.0;

    tod_msgs::ColoredPolygon msg;
    msg.header.frame_id = _planningFrame;
    msg.header.stamp = ros::Time::now();
    auto addPtToPolygon = [&](const State &state, const bool collisionFree, const auto &calcEdgeFcn) {
        double ptX, ptY;
        calcEdgeFcn(state.X, state.Y, state.Theta, _treeVehParams.LengthFrontBumper, _treeVehParams.Width, ptX, ptY);
        tod_msgs::ColoredPoint& cpt = msg.points.emplace_back();
        cpt.color = (collisionFree) ? GREEN : RED;
        cpt.point.x = (float) ptX;
        cpt.point.y = (float) ptY;
    };

    const TrajectoryTree& tree = _trajectoryPlanner->get_tree();
    for (int tjIdx = 0; tjIdx < tree.size(); ++tjIdx) {
        const Trajectory& tj = tree.at(tjIdx);
        bool putRightEdge = (tjIdx < tree.size() / 2);
        const auto& calcEdgeFcn =
            (putRightEdge) ? tod_helper::Vehicle::Model::calc_vehicle_front_right_edge
                           : tod_helper::Vehicle::Model::calc_vehicle_front_left_edge;
        bool addAllStates = (tjIdx == 0 || tjIdx == tree.size()-1);
        if (addAllStates) {
            int nofPtsBefore = (int) msg.points.size();
            for (const State &state : tj.States) {
                addPtToPolygon(state, tj.CollisionFree, calcEdgeFcn);
            }
            bool reverseAddedStates = (tjIdx == tree.size()-1);
            if (reverseAddedStates) {
                std::reverse(msg.points.begin() + nofPtsBefore, msg.points.end());
            }
        } else {
            addPtToPolygon(tj.States.back(), tj.CollisionFree, calcEdgeFcn);
        }
    }

    _pubPredictedPolygon.publish(msg);
}

void SharedVelocityController::publish_log(tod_shared_control::SvcLog &log) {
    log.header.stamp = ros::Time::now();
    _pubLog.publish(log);
}

void SharedVelocityController::publish_and_log_trajectory_tree(tod_shared_control::SvcLog &log) {
    bool PUBLISH_DIMENSIONS{false};
    visualization_msgs::MarkerArray msg;
    int id{0};
    static std_msgs::ColorRGBA RED; RED.a = 1.0; RED.r = 1.0;
    static std_msgs::ColorRGBA GREEN; GREEN.a = 1.0; GREEN.g = 1.0;
    for (const Trajectory& tj : _trajectoryPlanner->get_tree()) {
        visualization_msgs::Marker& marker = msg.markers.emplace_back();
        marker.header.frame_id = _planningFrame;
        marker.header.stamp = ros::Time::now();
        marker.pose.orientation.w = 1.0;
        marker.id = id++;
        marker.scale.x = 0.0125;
        marker.color = (tj.CollisionFree) ? GREEN : RED;
        if (PUBLISH_DIMENSIONS) {
            marker.type = visualization_msgs::Marker::LINE_LIST;
            int addEvery{4};
            for (int stIdx = 0; stIdx < tj.States.size(); stIdx += addEvery) {
                const State& state = tj.States.at(stIdx);
                double flX, flY, frX, frY, rlX, rlY, rrX, rrY;
                tod_helper::Vehicle::Model::calc_vehicle_front_edges(
                    state.X, state.Y, state.Theta,_coreVehParams->get_distance_front_bumper(),
                    _coreVehParams->get_width(), flX, flY, frX, frY);
                tod_helper::Vehicle::Model::calc_vehicle_rear_edges(
                    state.X, state.Y, state.Theta,_coreVehParams->get_distance_rear_bumper(),
                    _coreVehParams->get_width(), rlX, rlY, rrX, rrY);

                geometry_msgs::Point& leftFirst = marker.points.emplace_back();
                leftFirst.x = rlX; leftFirst.y = rlY;
                geometry_msgs::Point& leftSecond = marker.points.emplace_back();
                leftSecond.x = flX; leftSecond.y = flY;

                geometry_msgs::Point& frontFirst = marker.points.emplace_back();
                frontFirst.x = flX; frontFirst.y = flY;
                geometry_msgs::Point& frontSecond = marker.points.emplace_back();
                frontSecond.x = frX; frontSecond.y = frY;

                geometry_msgs::Point& rightFirst = marker.points.emplace_back();
                rightFirst.x = frX; rightFirst.y = frY;
                geometry_msgs::Point& rightSecond = marker.points.emplace_back();
                rightSecond.x = rrX; rightSecond.y = rrY;

                geometry_msgs::Point& rearFirst = marker.points.emplace_back();
                rearFirst.x = rrX; rearFirst.y = rrY;
                geometry_msgs::Point& rearSecond = marker.points.emplace_back();
                rearSecond.x = rlX; rearSecond.y = rlY;
            }
        } else {
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            for (const State& state : tj.States) {
                geometry_msgs::Point& pt = marker.points.emplace_back();
                pt.x = state.X;
                pt.y = state.Y;
            }
        }
    }
    if (_logTrajectoriyTreeTrajectories) {
        ros::Time t0 = ros::Time::now();
        // log.trajsMarker = msg;
        {
            // allocate memory
            log.trajsCollisionFree = std::vector<unsigned char>(_trajectoryPlanner->get_tree_size(), 0);
            log.trajsSafeProgress = std::vector<float>(_trajectoryPlanner->get_tree_size(), 0.0f);
            using tod_shared_control::FloatArray;
            FloatArray tmpStateArray;
            tmpStateArray.data = std::vector<float>(_trajectoryPlanner->get_trajectory_size(), 0.0f);
            log.trajsXPoses = std::vector<FloatArray>(_trajectoryPlanner->get_tree_size(), tmpStateArray);
            log.trajsYPoses = std::vector<FloatArray>(_trajectoryPlanner->get_tree_size(), tmpStateArray);
            log.trajsHeadings = std::vector<FloatArray>(_trajectoryPlanner->get_tree_size(), tmpStateArray);
            log.trajsSteeringAngles = std::vector<FloatArray>(_trajectoryPlanner->get_tree_size(), tmpStateArray);
            log.trajsVelocities = std::vector<FloatArray>(_trajectoryPlanner->get_tree_size(), tmpStateArray);
            log.trajsProgressValues = std::vector<FloatArray>(_trajectoryPlanner->get_tree_size(), tmpStateArray);
            FloatArray tmpInputArray;
            tmpInputArray.data = std::vector<float>(_trajectoryPlanner->get_trajectory_size()-1, 0.0f);
            log.trajsSteeringRates = std::vector<FloatArray>(_trajectoryPlanner->get_tree_size(), tmpInputArray);
            log.trajsLonAccs = std::vector<FloatArray>(_trajectoryPlanner->get_tree_size(), tmpInputArray);
        }
        for (int tIdx = 0; tIdx < _trajectoryPlanner->get_tree_size(); ++tIdx) {
            const Trajectory& tj = _trajectoryPlanner->get_tree().at(tIdx);
            log.trajsCollisionFree.at(tIdx) = tj.CollisionFree;
            log.trajsSafeProgress.at(tIdx) = tj.SafeProgress;
            for (int sIdx = 0; sIdx < _trajectoryPlanner->get_trajectory_size(); ++sIdx) {
                log.trajsXPoses.at(tIdx).data.at(sIdx) = tj.States.at(sIdx).X;
                log.trajsYPoses.at(tIdx).data.at(sIdx) = tj.States.at(sIdx).Y;
                log.trajsHeadings.at(tIdx).data.at(sIdx) = tj.States.at(sIdx).Theta;
                log.trajsSteeringAngles.at(tIdx).data.at(sIdx) = tj.States.at(sIdx).Delta;
                log.trajsVelocities.at(tIdx).data.at(sIdx) = tj.States.at(sIdx).Velocity;
                log.trajsProgressValues.at(tIdx).data.at(sIdx) = tj.States.at(sIdx).Progress;
                if (sIdx < _trajectoryPlanner->get_trajectory_size() - 1) {
                    log.trajsSteeringRates.at(tIdx).data.at(sIdx) = tj.Inputs.at(sIdx).DDelta;
                    log.trajsLonAccs.at(tIdx).data.at(sIdx) = tj.Inputs.at(sIdx).Acceleration;
                }
            }
        }
        double dt = (ros::Time::now() - t0).toSec();
        if (dt > ros::Duration(0.005).toSec()) {
            ROS_WARN("%s: logging trajectory tree trajectories took %.2f ms - consider to disable logging or "
                     "decrease tree complexity", _nn.c_str(), dt*1e3);
        }
    }
    _pubTrajectoryTree.publish(msg);
}

} // namespace tod_shared_control
