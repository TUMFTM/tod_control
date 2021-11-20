// Copyright 2021 Schimpe
#include "ForceFeedbackController.h"

ForceFeedbackController::ForceFeedbackController(ros::NodeHandle &nh)
    : _nh{nh}, _vehicleParams{_nh} {
    std::string vehicleDataTopic{"/Operator/VehicleBridge/vehicle_data"};
    _subscribers[vehicleDataTopic] = _nh.subscribe<tod_msgs::VehicleData>(
        vehicleDataTopic, 5, [this](const tod_msgs::VehicleDataConstPtr &msg) {
            _vehicleDataMsg = msg;
        });

    std::string ctrlCmdTopic{"/Operator/InputDevices/joystick"};
    _subscribers[ctrlCmdTopic] = _nh.subscribe<sensor_msgs::Joy>(
        ctrlCmdTopic, 5, [this](const sensor_msgs::JoyConstPtr &msg) {
            _operatorSWA = msg->axes[joystick::AxesPos::STEERING] * _vehicleParams.get_max_swa_rad();
        });

    _pubForceFeedback = _nh.advertise<std_msgs::Float64>("force_feedback", 5);

    double Kp{0.4};
    _nh.getParam(ros::this_node::getName() + "/Kp", Kp);
    _piCtrler = std::make_unique<PIController>(Kp, 0.0);

    _nh.getParam(ros::this_node::getName() + "/InvertSteeringInGearReverse", _invertSteeringInGearReverse);
}

void ForceFeedbackController::run() {
    ros::Rate r(20);
    while (ros::ok()) {
        ros::spinOnce();
        if (_vehicleDataMsg) {
            double currentWheelPosition =
                (_invertSteeringInGearReverse && _vehicleDataMsg->gearPosition == eGearPosition::GEARPOSITION_REVERSE)
                    ? -_operatorSWA : _operatorSWA;
            double desiredWheelPosition = _vehicleDataMsg->steeringWheelAngle;
            double ffValue = _piCtrler->get_input(currentWheelPosition, desiredWheelPosition);
            ffValue = std::clamp(ffValue, -0.8, 0.8); // -1.0, 1.0
            if (ros::Time::now() >= _vehicleDataMsg->header.stamp + ros::Duration(0.5)) {
                // no force feedback when vehicle data is old
                _vehicleDataMsg.reset();
                ffValue = 0.0;
            }
            std_msgs::Float64 msg;
            msg.data = ffValue;
            _pubForceFeedback.publish(msg);
        }
        r.sleep();
    }
}
