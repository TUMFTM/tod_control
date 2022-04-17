// Copyright 2020 Simon Hoffmann
#include "PathTrackingControl.h"
PathTrackingControl::PathTrackingControl(ros::NodeHandle& nodeHandle) : _nh(nodeHandle),
        _transformListener{tf2_ros::TransformListener(_tfBuffer)}, _nodeName(ros::this_node::getName()) {
    _vehParams = std::make_unique<tod_core::VehicleParameters>(_nh);
    bool debug = false;
    if (!_nh.getParam(_nodeName + "/debug", debug))
        ROS_ERROR("%s: Could not get parameter debug!", _nodeName.c_str());
    if (debug) // print ROS_DEBUG
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
            ros::console::notifyLoggerLevelsChanged();

    _purePursuit = std::make_unique<PurePursuit>(_nh, _vehParams);
    ros_init();
}

void PathTrackingControl::ros_init() {
    // subscriber
    _subOdometry = _nh.subscribe("odometry", 10, &PathTrackingControl::callback_odometry, this);
    _subTrajectory = _nh.subscribe("trajectory", 10, &PathTrackingControl::callback_trajectory, this);
    _subStatus = _nh.subscribe("status_msg", 1, &PathTrackingControl::callback_status_msg, this);

    // publisher
    _pubControlCmd = _nh.advertise<tod_msgs::PrimaryControlCmd>("primary_control_cmd", 1);
    _pubLogging = _nh.advertise<tod_pure_pursuit::ppLog>("ptc_logging", 1);

    // get ControlMode
    int mode;
    if (!_nh.getParam(ros::this_node::getName() + "/ControlMode", mode)) {
        ROS_ERROR("no control mode provided at %s. Shutting down node", ros::this_node::getName().c_str());
    }
    _desiredControlMode = mode;
}

void PathTrackingControl::run() {
    ros::Rate r(100);
    while (ros::ok() && _desiredControlMode != tod_msgs::Status::CONTROL_MODE_NONE) {
        ros::spinOnce();
        if (_vehParams->vehicle_id_has_changed())
            _vehParams->load_parameters();
        if (_trajectory.points.size() > 0 && _actualControlMode == _desiredControlMode) {
            if (_trajectory.header.frame_id != _poseRearAxle.header.frame_id) {
                ROS_ERROR("%s: Odometry(%s) and Trajectory(%s) provided in different frames", _nodeName.c_str(),
                    _poseRearAxle.header.frame_id.c_str(), _trajectory.header.frame_id.c_str());
                continue;
            }
            _purePursuit->calc_control_command(_trajectory, _poseRearAxle, _primaryControlCommand, _actualSpeed);
            _primaryControlCommand.header.stamp = ros::Time::now();
            _pubControlCmd.publish(_primaryControlCommand);
            publish_logging_output();
        }
        r.sleep();
    }
}

void PathTrackingControl::callback_trajectory(const tod_msgs::Trajectory& trajectory) {
    if (trajectory.points.size() <=1)
        return;
    _trajectory = trajectory;
    if (trajectory.child_frame_id != _purePursuit->get_rearAxleFrameId()) {
        geometry_msgs::TransformStamped tf = get_transform(_purePursuit->get_rearAxleFrameId(),
        _trajectory.child_frame_id);
        tod_helper::Trajectory::transform_child_frame(_trajectory, tf);
    }
}

void PathTrackingControl::callback_odometry(const nav_msgs::Odometry& odometry) {
    _actualSpeed = odometry.twist.twist.linear.x;
    _poseRearAxle.pose = odometry.pose.pose;
    if (odometry.child_frame_id != _purePursuit->get_rearAxleFrameId()) {
        geometry_msgs::TransformStamped tf = get_transform(_purePursuit->get_rearAxleFrameId(),
            odometry.child_frame_id);
        tod_helper::Trajectory::transform_child_frame(_poseRearAxle.pose, tf);
    }
    _poseRearAxle.header = odometry.header;
}

geometry_msgs::TransformStamped PathTrackingControl::get_transform(const std::string& srcFrame,
        const std::string& trgFrame) {
    geometry_msgs::TransformStamped transform;
    if (_tfBuffer.canTransform(trgFrame, srcFrame, ros::Time::now()))
        transform = _tfBuffer.lookupTransform(trgFrame, srcFrame, ros::Time::now());
    else
        ROS_ERROR("%s: cannot get tf from %s to %s", _nodeName.c_str(), srcFrame.c_str(), trgFrame.c_str());
    return transform;
}

void PathTrackingControl::callback_status_msg(const tod_msgs::Status &msg) {
    if (_actualControlMode != msg.vehicle_control_mode)
        _trajectory.points.clear();
    _actualControlMode = msg.vehicle_control_mode;
}

void PathTrackingControl::publish_logging_output() {
    tod_pure_pursuit::ppLog log;
    log.poseError = _purePursuit->poseError;
    log.timeErrorToClosestPose = _purePursuit->timeErrorToClosestPose;
    log.timeErrorToFirstPose = _purePursuit->timeErrorToFirstPose;
    log.closestWP = _purePursuit->closestWP;
    _pubLogging.publish(log);
}
