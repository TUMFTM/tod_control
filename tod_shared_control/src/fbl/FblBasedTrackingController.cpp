// Copyright 2021 Andreas Schimpe
#include "FblBasedTrackingController.h"

namespace tod_shared_control {

FblBasedTrackingController::FblBasedTrackingController(ros::NodeHandle& nodeHandle) :
    ControllerBase(nodeHandle),
    _vehicleParams{std::make_unique<tod_core::VehicleParameters>(_nh)},
    _trajHandler{std::make_unique<TrajectoryHandler>()} {
    _fb.disable_control_cmd_feedback();

    std::string trajectoryTopic{"trajectory"};
    _subscriberMap[trajectoryTopic] = _nh.subscribe<tod_msgs::Trajectory>(
                trajectoryTopic, 1, &TrajectoryHandler::trajectory_callback, _trajHandler.get());

    _pubLog = _nh.advertise<tod_shared_control::FblLog>("fbl_log", 1);
}

void FblBasedTrackingController::run() {
    ros::Rate controllerRate(50);

    while (ros::ok()) {
        controllerRate.sleep();
        ros::spinOnce();
        if (!_fb.in_shared_control_and_teleoperation()) {
            continue;
        }
        tod_msgs::PrimaryControlCmd cmd = execute_control(controllerRate.expectedCycleTime().toSec());
        publish_ctrl_cmd(cmd);
        check_cycle_time(controllerRate);
    }
}

tod_msgs::PrimaryControlCmd FblBasedTrackingController::execute_control(const double dt) {
    if (!_fb.feedback_complete() || !_trajHandler->has_trajectory()) {
        ROS_WARN_THROTTLE(1.0, "%s: lacking state feedback - has veh_data %d, odom %d, traj %d", _nn.c_str(),
                          _fb.vehicle_data() != nullptr, _fb.odometry() != nullptr, _trajHandler->has_trajectory());
        return tod_msgs::PrimaryControlCmd();
    }

    tod_shared_control::FblLog log;
    log.odometry = *_fb.odometry();

    double currentRWA = _vehicleParams->compute_rwa_from(_fb.vehicle_data()->steeringWheelAngle);
    BicycleState currentState(_fb.odometry()->pose.pose.position.x, _fb.odometry()->pose.pose.position.y,
                              tf2::getYaw(_fb.odometry()->pose.pose.orientation), currentRWA, _fb.vehicle_data()->longitudinalSpeed);
    log.currentRWA = currentRWA;
    log.currentVelocity = currentState.Cmd.Velocity;

    double lookahead{2.0}, trackingError;
    tod_msgs::TrajectoryPoint trackingPt = _trajHandler->get_tracking_point(*_fb.odometry(), lookahead, trackingError);
    tod_msgs::TrajectoryPoint closestPt = _trajHandler->get_tracking_point(*_fb.odometry(), 0.0, trackingError);
    BicycleState desiredState(trackingPt.pose.pose.position.x, trackingPt.pose.pose.position.y,
                              tf2::getYaw(trackingPt.pose.pose.orientation), 0.0, closestPt.twist.twist.linear.x);
    log.lookahead = lookahead;
    log.trackingPoint = trackingPt;
    log.trackingError = trackingError;
    log.desiredVelocity = desiredState.Cmd.Velocity;

    double rwaCmd = _fbl.compute_steering_ctrl_cmd(currentState, desiredState, _vehicleParams->get_wheel_base(), dt,
                                                   log.lateralError, log.headingError);

    tod_msgs::PrimaryControlCmd cmd;
    cmd.steeringWheelAngle = _vehicleParams->compute_swa_from(rwaCmd);
    cmd.velocity = desiredState.Cmd.Velocity;
    cmd.acceleration = 0.0;

    log.ctrlCmd = cmd;
    publish_log(log);

    return cmd;
}

void FblBasedTrackingController::publish_log(tod_shared_control::FblLog &log) {
    log.header.stamp = ros::Time::now();
    _pubLog.publish(log);
}

}; // namespace tod_shared_control
