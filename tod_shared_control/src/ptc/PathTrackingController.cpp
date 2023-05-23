// Copyright 2021 Andreas Schimpe
#include "PathTrackingController.h"

namespace tod_shared_control {

PathTrackingController::PathTrackingController(ros::NodeHandle& nodeHandle) :
    ControllerBase(nodeHandle),
    _trajHandler{std::make_shared<TrajectoryHandler>()},
    _vehicleParams{std::make_unique<tod_core::VehicleParameters>(_nh)},
    _ctrlerParams{std::make_shared<PtcSolver::Parameters>()} {
    auto mySolver = std::make_unique<PtcSolver>(_ctrlerParams);
    if (!mySolver->ready()) {
        ROS_ERROR("%s: PtcSolver setup failed - terminating!", _nn.c_str());
        return;
    }

    _fb.disable_control_cmd_feedback();

    _ctrlerParams->steeringMax = double(_vehicleParams->get_max_rwa_rad());
    _ctrlerParams->steeringRateMax = tod_helper::Vehicle::Model::deg2rad(60.0);
    _ctrlerParams->accelerationMin = -10.0; // m/s^2
    _ctrlerParams->accelerationMax = 5.0;

    std::string trajectoryTopic{"trajectory"};
    _subscriberMap[trajectoryTopic] = _nh.subscribe<tod_msgs::Trajectory>(
        trajectoryTopic, 1, &TrajectoryHandler::trajectory_callback, _trajHandler.get());

    // TODO(Andi): put this into ptc log msg
    _pubReference = _nh.advertise<visualization_msgs::MarkerArray>("reference", 1);
    _pubPredictions = _nh.advertise<visualization_msgs::MarkerArray>("predictions", 1);
    _pubSolverTime = _nh.advertise<std_msgs::Float32>("solver_time", 1);
    _pubTrackingError = _nh.advertise<std_msgs::Float32>("tracking_error", 1);
}

void PathTrackingController::run() {
    ros::Rate controllerRate(20);
    while (ros::ok()) {
        controllerRate.sleep();
        ros::spinOnce();
        if (!_fb.in_shared_control()) {
            continue;
        }
        if (_fb.started_teleoperation()) {
            _solver = std::make_unique<PtcSolver>(_ctrlerParams);
            if (!_solver->cycle_time_consistent(controllerRate.expectedCycleTime().toSec())) {
                ROS_WARN("%s: ctrler cycle time (%.2fs) and mpc sample time (%.2fs) are inconsistent",
                         _nn.c_str(), controllerRate.expectedCycleTime().toSec(), _solver->get_sampling_time());
            }
        } else if (_fb.terminated_teleoperation()) {
            _solver.reset();
            _fb.reset();
        }
        if (_fb.in_teleoperation()) {
            tod_msgs::PrimaryControlCmd ptcCmd = execute_control();
            publish_ctrl_cmd(ptcCmd);
        }
        check_cycle_time(controllerRate);
        _fb.update_previous_in_teleoperation();
    }
}

tod_msgs::PrimaryControlCmd PathTrackingController::execute_control() {
    const ros::Time t0 = ros::Time::now();
    if (!_fb.feedback_complete() || !_trajHandler->has_trajectory()) {
        ROS_WARN_THROTTLE(1.0, "%s: lacking state feedback - has veh_data %d, odom %d, traj %d", _nn.c_str(),
                          _fb.vehicle_data() != nullptr, _fb.odometry() != nullptr, _trajHandler->has_trajectory());
        return tod_msgs::PrimaryControlCmd();
    }

    // get data and solve
    BicycleState stateInit = get_initial_state();
    double trackingError{0.0};
    std::vector<BicycleState> statesRef = get_reference_states(stateInit, trackingError);
    BicycleCommand ctrlerCommand;
    std::vector<BicycleState> statePredictions;
    SolverBase::AcadosDebug acadosDebug;
    bool cmdOkay{true};
    if (!reached_final_waypoint(stateInit, statesRef.back())) {
        cmdOkay = _solver->solve(statesRef, stateInit, ctrlerCommand, statePredictions, acadosDebug);
    }

    // set and publish control command
    tod_msgs::PrimaryControlCmd ptcCmd;
    if (cmdOkay) {
        ptcCmd.steeringWheelAngle = _vehicleParams->compute_swa_from(ctrlerCommand.Steering);
        ptcCmd.velocity = float(ctrlerCommand.Velocity);
        ptcCmd.acceleration = float(ctrlerCommand.input.Acceleration);
    } else {
        ROS_WARN("%s: Solver failed - not setting control command", _nn.c_str());
    }

    acadosDebug.solver_time_ms = (ros::Time::now() - t0).toSec() * 1e3;
    publish_reference_and_predictions(statesRef, statePredictions);
    publish_solver_time(acadosDebug);
    publish_tracking_error(trackingError);

    return ptcCmd;
}

BicycleState PathTrackingController::get_initial_state() {
    BicycleState stateInit;
    stateInit.PosX = _fb.odometry()->pose.pose.position.x;
    stateInit.PosY = _fb.odometry()->pose.pose.position.y;
    stateInit.Heading = tf2::getYaw(_fb.odometry()->pose.pose.orientation);
    stateInit.Cmd.Steering = _vehicleParams->compute_rwa_from(_fb.vehicle_data()->steeringWheelAngle);
    stateInit.Cmd.Velocity = _fb.vehicle_data()->longitudinalSpeed;
    return stateInit;
}

std::vector<BicycleState> PathTrackingController::get_reference_states(const BicycleState &stateInit, double &outTrackingError) {
    tod_msgs::Trajectory waypoints = _trajHandler->get_next_waypoints(
        *_fb.odometry(), _solver->get_sampling_time(), _solver->get_prediction_horizon(), outTrackingError);
    std::vector<BicycleState> stateReference;
    stateReference.push_back(stateInit);
    for (const auto &wp : waypoints.points) {
        auto& st = stateReference.emplace_back();
        st.PosX = wp.pose.pose.position.x;
        st.PosY = wp.pose.pose.position.y;
        st.Heading = tf2::getYaw(wp.pose.pose.orientation);
    }
    return stateReference;
}

void PathTrackingController::publish_reference_and_predictions(const std::vector<BicycleState> &reference,
                                                               const std::vector<BicycleState> &predictions) {
    std_msgs::ColorRGBA whiteColor, redColor;
    whiteColor.r = whiteColor.g = whiteColor.b = whiteColor.a = 1.0;
    redColor.r = redColor.a = 1.0;
    visualization_msgs::MarkerArray refMarker =
        tod_helper::Trajectory::to_marker_array(states_to_trajectory(reference), whiteColor);
    visualization_msgs::MarkerArray predMarker =
        tod_helper::Trajectory::to_marker_array(states_to_trajectory(predictions), redColor);
    _pubReference.publish(refMarker);
    _pubPredictions.publish(predMarker);
}

void PathTrackingController::publish_solver_time(const SolverBase::AcadosDebug &acadosDebug) {
    std_msgs::Float32 msg;
    msg.data = acadosDebug.solver_time_ms;
    _pubSolverTime.publish(msg);
}

void PathTrackingController::publish_tracking_error(const double trackingError) {
    std_msgs::Float32 msg;
    msg.data = float(trackingError);
    _pubTrackingError.publish(msg);
}

tod_msgs::Trajectory PathTrackingController::states_to_trajectory(const std::vector<BicycleState> &states) {
    tod_msgs::Trajectory trajectory;
    trajectory.header.frame_id = _trajHandler->get_frame_id();
    trajectory.child_frame_id = "base_footprint";
    for (const auto &state : states) {
        auto& tp = trajectory.points.emplace_back();
        tp.pose.pose.position.x = state.PosX;
        tp.pose.pose.position.y = state.PosY;
        tp.twist.twist.linear.x = state.Cmd.Velocity;
    }
    return trajectory;
}

}; // namespace tod_shared_control
