// Copyright 2020 Andreas Schimpe
#include "ModelPredictiveController.h"

namespace tod_shared_control {

SharedController::SharedController(ros::NodeHandle& nodeHandle) :
    ControllerBase{nodeHandle},
    _planningFrame{"base_footprint"},
    _objectsHandler{_nh, _planningFrame} {
    _ctrlerParams = std::make_shared<tod_shared_control::MpcSolver::Parameters>(_nh);
    auto mySolver = std::make_unique<tod_shared_control::MpcSolver>(_ctrlerParams);
    if (!mySolver->ready()) {
        ROS_ERROR("%s: Solver setup failed - terminating!", _nn.c_str());
        return;
    }

    _pubPredictedPolygon = _nh.advertise<tod_msgs::ColoredPolygon>("predicted_polygon", 1);
    _pubAvoidedObstacles = _nh.advertise<tod_msgs::ObjectList>("avoided_obstacles", 1);
    _pubMpcLog = _nh.advertise<tod_shared_control::MpcLog>("mpc_log", 1);

    std::string vehicleId{""};
    _nh.getParam("/vehicleID", vehicleId);

    // controller penalties and constraints
    if (vehicleId == "tum-q7") {
        _ctrlerParams->steeringRateMax = tod_helper::Vehicle::Model::deg2rad(60.0);
        _ctrlerParams->accelerationMin = -3.5;
        _ctrlerParams->accelerationMax = 2.0;
        _ctrlerParams->steeringRateCost = 0.01;
        _ctrlerParams->accelerationCost = 0.01;
        _ctrlerParams->steeringTrackingCost = 350;
        _ctrlerParams->velocityTrackingCost = 1.0;
        _ctrlerParams->cmdCostDecay = 0.05;
    } else if (vehicleId == "rc-car") {
        _ctrlerParams->steeringRateMax = tod_helper::Vehicle::Model::deg2rad(60.0);
        _ctrlerParams->accelerationMin = -2.0;
        _ctrlerParams->accelerationMax = 8.0;
        _ctrlerParams->steeringRateCost = 0.25;
        _ctrlerParams->accelerationCost = 0.005;
        _ctrlerParams->steeringTrackingCost = 350;
        _ctrlerParams->velocityTrackingCost = 4.5;
        // 0.00 for constant steering intervention cost
        // 0.05 for decaying steering intervention cost
        _ctrlerParams->cmdCostDecay = 0.05;
    } else {
        ROS_ERROR("unknown vehicle id in mpc");
    }
}

void SharedController::run() {
    ros::Rate controllerRate(20);
    while (ros::ok()) {
        controllerRate.sleep();
        ros::spinOnce();
        if (!_fb.in_shared_control()) {
            continue;
        }
        if (_fb.started_teleoperation()) {
            _solver = std::make_unique<tod_shared_control::MpcSolver>(_ctrlerParams);
            if (!_solver->cycle_time_consistent(controllerRate.expectedCycleTime().toSec())) {
                ROS_WARN("%s: ctrler cycle time (%.2fs) and mpc sample time (%.2fs) are inconsistent",
                         _nn.c_str(), controllerRate.expectedCycleTime().toSec(), _solver->get_sampling_time());
            }
        } else if (_fb.terminated_teleoperation()) {
            _solver.reset();
            _objectsHandler.reset();
            _fb.reset();
        }
        if (_fb.in_teleoperation()) {
            tod_msgs::PrimaryControlCmd ctrlCmd = execute_control(controllerRate);
            publish_ctrl_cmd(ctrlCmd);
        }
        check_cycle_time(controllerRate);
        _fb.update_previous_in_teleoperation();
    }
}

tod_msgs::PrimaryControlCmd SharedController::execute_control(const ros::Rate &controllerRate) {
    if (!_fb.feedback_complete()) {
        ROS_WARN_THROTTLE(1.0, "%s: lacking state feedback - has veh_data %d, ctrl cmd %d, odom %d", _nn.c_str(),
                          _fb.vehicle_data() != nullptr, _fb.primary_ctrl_cmd() != nullptr, _fb.odometry() != nullptr);
        return tod_msgs::PrimaryControlCmd();
    }

    // solve
    tod_shared_control::BicycleState refSt = get_reference_state();
    tod_shared_control::BicycleState initSt = get_initial_state();
    std::vector<tod_msgs::ObjectData> objects = _objectsHandler.get_n_nearest_objects_from_xy(
        _solver->get_nof_obstacles(), initSt.PosX, initSt.PosY, _fb.vehicle_data()->gearPosition);
    tod_shared_control::BicycleCommand ctrlerCommand;
    tod_msgs::ColoredPolygon predictedPolygon;
    SolverBase::AcadosDebug acadosDebug;
    std::vector<double> xTraj, uTraj;
    bool ret = _solver->solve(refSt, initSt, objects, predictedPolygon, ctrlerCommand,
                              xTraj, uTraj, acadosDebug);

    // set control command
    tod_msgs::PrimaryControlCmd ctrlCmd;
    if (!ret) {
        ROS_WARN("%s: solver failed - setting control command to current state",
                 _nn.c_str());
        ctrlCmd.steeringWheelAngle = _fb.vehicle_data()->steeringWheelAngle;
        ctrlCmd.velocity = _fb.vehicle_data()->longitudinalSpeed;
    } else {
        ctrlCmd.steeringWheelAngle = _ctrlerParams->vehParams->compute_swa_from(ctrlerCommand.Steering);
        ctrlCmd.velocity = std::max(ctrlerCommand.Velocity, 0.0);
        ctrlCmd.velocity = std::min(ctrlCmd.velocity, float(refSt.Cmd.Velocity));
    }
    double currentVelocity = _fb.vehicle_data()->longitudinalSpeed;
    ctrlCmd.acceleration = (ctrlCmd.velocity - currentVelocity) / controllerRate.expectedCycleTime().toSec();

    // publish debug
    publish_avoided_obstacles(objects);
    publish_predicted_polygon(predictedPolygon);

    // tf polygon and objects into odom frame for log
    geometry_msgs::TransformStamped tf = get_tf(_planningFrame, _fb.odometry()->header.frame_id);
    tod_helper::ObjectList::transform(objects, tf);
    tod_helper::ColoredPolygon::transform(predictedPolygon, tf);

    // set log
    tod_shared_control::MpcLog logMsg;
    logMsg.N = acadosDebug.N;
    logMsg.nx = acadosDebug.nx;
    logMsg.nu = acadosDebug.nu;
    logMsg.xTraj = xTraj;
    logMsg.uTraj = uTraj;
    logMsg.deltaMin = - _ctrlerParams->vehParams->get_max_rwa_rad();
    logMsg.deltaMax = _ctrlerParams->vehParams->get_max_rwa_rad();
    logMsg.ddeltaMin = - _ctrlerParams->steeringRateMax;
    logMsg.ddeltaMax = _ctrlerParams->steeringRateMax;
    logMsg.accelerationMin = _ctrlerParams->accelerationMin;
    logMsg.accelerationMax = _ctrlerParams->accelerationMax;
    logMsg.deltaRef = refSt.Cmd.Steering;
    logMsg.odometry = *_fb.odometry();
    logMsg.deltaCtrler = ctrlerCommand.Steering;
    logMsg.velocityRef = refSt.Cmd.Velocity;
    logMsg.ddeltaCtrler = ctrlerCommand.input.SteeringRate;
    logMsg.accelerationCtrler = ctrlCmd.acceleration;
    logMsg.acados_status = acadosDebug.status;
    logMsg.velocityCtrler = ctrlCmd.velocity;
    logMsg.acados_solver_time_ms = acadosDebug.solver_time_ms;
    logMsg.acados_sqp_iterations = acadosDebug.sqp_iterations;
    logMsg.objectList = objects;
    logMsg.predictedPolygon = predictedPolygon;
    publish_log(logMsg);

    return ctrlCmd;
}

void SharedController::publish_avoided_obstacles(const std::vector<tod_msgs::ObjectData> objects) {
    tod_msgs::ObjectList msg; // as object list
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = _planningFrame;
    msg.objectList = objects;
    _pubAvoidedObstacles.publish(msg);
}

void SharedController::publish_predicted_polygon(tod_msgs::ColoredPolygon &predictedPolygon) {
    predictedPolygon.header.stamp = ros::Time::now();
    predictedPolygon.header.frame_id = _planningFrame;
    _pubPredictedPolygon.publish(predictedPolygon);
}

void SharedController::publish_log(tod_shared_control::MpcLog &logMsg) {
    logMsg.header.stamp = ros::Time::now();
    logMsg.header.frame_id = _fb.odometry()->header.frame_id;
    _pubMpcLog.publish(logMsg);
}

tod_shared_control::BicycleState SharedController::get_reference_state() {
    tod_shared_control::BicycleState stateRef;
    stateRef.Cmd.Steering = _ctrlerParams->vehParams->compute_rwa_from(_fb.primary_ctrl_cmd()->steeringWheelAngle);
    stateRef.Cmd.Velocity = _fb.primary_ctrl_cmd()->velocity;
    return stateRef;
}

tod_shared_control::BicycleState SharedController::get_initial_state() {
    tod_shared_control::BicycleState stateInit;
    stateInit.Cmd.Steering = _ctrlerParams->vehParams->compute_rwa_from(_fb.vehicle_data()->steeringWheelAngle);
    stateInit.Cmd.Velocity = _fb.vehicle_data()->longitudinalSpeed;
    if (_planningFrame != "base_footprint") {
        if (_planningFrame != _fb.odometry()->header.frame_id)
            ROS_WARN("%s: planning frame is %s, odom frame is %s", _nn.c_str(),
                     _planningFrame.c_str(), _fb.odometry()->header.frame_id.c_str());
        stateInit.PosX = _fb.odometry()->pose.pose.position.x;
        stateInit.PosY = _fb.odometry()->pose.pose.position.y;
        stateInit.Heading = tod_helper::Geometry::get_yaw_from_quaternion(_fb.odometry()->pose.pose.orientation);
    }
    return stateInit;
}

} // namespace tod_shared_control
