// Copyright 2021 Schimpe
#pragma once
#include <ros/ros.h>
#include <string>
#include <memory>
#include <map>
#include <tod_msgs/VehicleData.h>
#include <sensor_msgs/Joy.h>
#include <tod_msgs/joystickConfig.h>
#include <tod_helper/vehicle/Parameters.h>
#include "tod_msgs/VehicleEnums.h"
#include <std_msgs/Float64.h>

class PIController {
public:
    PIController(const double Kp, const double Ki, const double maxIntegral = 0.5,
                 const ros::Duration resetDurationSec = ros::Duration{0.5})
        : _Kp{Kp}, _Ki{Ki}, _Imax{maxIntegral}, _resetDurationSec{resetDurationSec} {}
    ~PIController() {}

    double get_input(const double actual, const double desired) {
        reset_integral_if_reset_duration_passed();
        ros::Duration elapsed = ros::Time::now() - _lastCalculation;
        double error = actual - desired;
        _Ierror += elapsed.toSec() * error;
        _Ierror = std::clamp(_Ierror, -_Imax, _Imax);
        double input = _Kp * error + _Ki * _Ierror;
        _lastCalculation = ros::Time::now();
        return input;
    }

private:
    double _Kp, _Ki;
    double _Imax, _Ierror{0.0};
    ros::Duration _resetDurationSec;
    ros::Time _lastCalculation{0.0};

    void reset_integral_if_reset_duration_passed() {
        if (ros::Time::now() >= _lastCalculation + _resetDurationSec) {
            _Ierror = 0.0;
            _lastCalculation = ros::Time::now();
        }
    }
};

class ForceFeedbackController {
public:
    explicit ForceFeedbackController(ros::NodeHandle &nh);
    ~ForceFeedbackController() {}
    void run();

private:
    ros::NodeHandle& _nh;
    tod_helper::Vehicle::Parameters _vehicleParams;
    std::unique_ptr<PIController> _piCtrler{nullptr};
    std::map<std::string, ros::Subscriber> _subscribers;
    bool _invertSteeringInGearReverse{false};
    ros::Publisher _pubForceFeedback;
    tod_msgs::VehicleDataConstPtr _vehicleDataMsg{nullptr};
    double _operatorSWA{0.0};
};
