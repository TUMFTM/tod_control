// Copyright 2020 Simon Hoffmann
#include "SafeTrajCreator.h"


SafeTrajCreator::SafeTrajCreator(ros::NodeHandle &nodeHandle) :
        _transformListener{tf2_ros::TransformListener(_tfBuffer)},
        _nh(nodeHandle) { }

void SafeTrajCreator::run() {
    init_ros_interface();
    init_param_sets();

    _sweptPathCreator = std::make_unique<SweptPathCreator>(_vehParams, _corridorParams);
    _latPlanner = LatPlannerFactory::create(LateralBehavior::CLOTHOID, _vehParams, _trajParams->get_segment_length());
    _longPlanner = LongPlannerFactory::create(DecelerationProfile::TIME_LINEAR);

    ros::Rate r(100);
    while (ros::ok()) {
        ros::spinOnce();
        if (_vehParams->vehicle_id_has_changed())
            _vehParams->load_parameters();

        if (_poseIsSet && in_safe_corridor_control() && _newControlCommand) {
            static SMState _state;
            // Create Trajectory
            tod_msgs::Trajectory traj = _latPlanner->calculate_path(*_ctrlCmd, _trajParams->get_frame_id(),
                _trajParams->get_child_frame_id(), calc_max_path_length(_ctrlCmd->velocity,
                calc_offset_to_deceleration(_ctrlCmd->velocity)));
            _longPlanner->apply_velocity_profile(traj, _ctrlCmd->velocity,
                calc_offset_to_deceleration(_ctrlCmd->velocity), _speedPlannerParams->desiredDeceleration);

            // Create Swept Path
            geometry_msgs::PolygonStamped corridor;
            _sweptPathCreator->create_corridor(corridor, traj);

            // Get maximum Gate state on Operator/Vehicle side
            SMState maxState = std::max(_oGate, _vGate);

            // Publish
            if (_state != SMState::CLOSED) {
                _trajectoryPub.publish(traj);
                _corridorPub.publish(tod_helper::ColoredPolygon::to_colored_polygon(corridor,
                    tod_safety_monitoring::get_color(std::max(_oGate, _vGate))));
            }
            _newControlCommand = false;
            _state = maxState; //update here to publish "red" corridor at least once
        }
        r.sleep();
    }
}

void SafeTrajCreator::init_param_sets() {
    _corridorParams = std::make_shared<SweptPathParams>(_nh);
    _vehParams = std::make_shared<tod_core::VehicleParameters>(_nh);
    _speedPlannerParams = std::make_unique<SpeedPlannerParams>(_nh);
    _trajParams = std::make_unique<PathCreatorParams>(_nh);
}

void SafeTrajCreator::init_ros_interface() {
    _trajectoryPub = _nh.advertise<tod_msgs::Trajectory>("trajectory_control_command", 1);
    _corridorPub = _nh.advertise<tod_msgs::ColoredPolygon>("corridor", 1);
    _inputSubs = _nh.subscribe("/primary_control_cmd", 1, &SafeTrajCreator::callback_control_msg, this);
    _poseSubs = _nh.subscribe("/odometry", 1, &SafeTrajCreator::callback_pose, this);
    _statusSubs = _nh.subscribe("/status_msg", 1, &SafeTrajCreator::callback_status, this);
    _vGateStateSubs = _nh.subscribe("gate_state_vehicle", 1, &SafeTrajCreator::callback_gate_state_vehicle, this);
    _oGateStateSubs = _nh.subscribe("gate_state_operator", 1, &SafeTrajCreator::callback_gate_state_operator, this);
}

void SafeTrajCreator::callback_gate_state_vehicle(const tod_safety_monitoring::GateState& msg) {
    _vGate = static_cast<SMState>(msg.state);
}

void SafeTrajCreator::callback_gate_state_operator(const tod_safety_monitoring::GateState& msg) {
    _oGate = static_cast<SMState>(msg.state);
}

void SafeTrajCreator::callback_pose(const nav_msgs::Odometry &msg) {
    double yaw = tod_helper::Geometry::get_yaw_from_quaternion(msg.pose.pose.orientation);

    _latPlanner->update_pose(msg.pose.pose.position.x - _vehParams->get_distance_rear_axle() *std::cos(yaw),
        msg.pose.pose.position.y - _vehParams->get_distance_rear_axle() *std::sin(yaw), yaw);
    _longPlanner->set_stamp_of_ref_point(msg.header.stamp);
    _poseIsSet = true;
}

void SafeTrajCreator::callback_status(const tod_msgs::Status &msg) {
    _ctrlMode = msg.vehicle_control_mode;
}

void SafeTrajCreator::callback_control_msg(const tod_msgs::PrimaryControlCmdConstPtr msg) {
    _ctrlCmd = msg;
    _newControlCommand = true;
}

double SafeTrajCreator::calc_max_path_length(const double velocity, const double offset_to_deceleration) {
    return _longPlanner->get_braking_distance(velocity, offset_to_deceleration,
        _speedPlannerParams->desiredDeceleration) +
        _corridorParams->longitudinalOffset + _corridorParams->bufferFront +
        _trajParams->get_lookahead_distance(velocity);
}

double SafeTrajCreator::calc_offset_to_deceleration(const double velocity) {
    return velocity * _speedPlannerParams->timeOffsetToDeceleration;
}
