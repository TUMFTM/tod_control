// Copyright 2020 Simon Hoffmann
#pragma once
#include "sensor_msgs/Joy.h"
#include "tod_msgs/Status.h"
#include "tod_msgs/PrimaryControlCmd.h"
#include "tod_msgs/SecondaryControlCmd.h"
#include "tod_msgs/VehicleEnums.h"
#include "tod_msgs/joystickConfig.h"
#include <tod_helper/vehicle/Model.h>
#include <utility>
#include <stdio.h>
#include <vector>
#include <map>
#include <algorithm>
#include <ros/ros.h>
#include <tod_core/VehicleParameters.h>

class CommandCreator {
public:
    explicit CommandCreator(ros::NodeHandle &nodeHandle);
    ~CommandCreator() = default;
    void run();

private:
    ros::Subscriber _joystickSubs;
    ros::Subscriber _statusSubs;
    ros::Publisher _primaryControlPub, _secondaryControlPub;
    std::map<joystick::ButtonPos, int> _prevButtonState;
    uint8_t _status{tod_msgs::Status::TOD_STATUS_IDLE};
    tod_msgs::PrimaryControlCmd _primaryControlMsg;
    tod_msgs::SecondaryControlCmd _secondaryControlMsg;
    std::unique_ptr<tod_core::VehicleParameters> _vehParams;
    ros::NodeHandle _nh;

    bool _constraintSteeringRate{false};
    bool _invertSteeringInGearReverse{false};
    float _maxSpeedms{10};
    float _maxAcceleration{4};
    float _maxDeceleration{9};
    double _maxSteeringWheelAngleRate{15.0};
    bool _joystickInputSet{false};

    void init_control_messages();
    void callback_joystick_msg(const sensor_msgs::Joy::ConstPtr &msg);
    void callback_status_msg(const tod_msgs::Status &msg);
    void calculate_steering_wheel_angle(tod_msgs::PrimaryControlCmd &out, const std::vector<float> &axes);
    void calculate_desired_velocity(tod_msgs::PrimaryControlCmd &out, const sensor_msgs::Joy::ConstPtr &msg,
            const int gear);
    void set_gear(tod_msgs::SecondaryControlCmd &out, const std::vector<int> &buttonState,
            const float &currentVelocity);
    void set_indicator(tod_msgs::SecondaryControlCmd &out, const std::vector<int> &buttonState);
    void set_light(tod_msgs::SecondaryControlCmd &out, const std::vector<int> &buttonState);
    void set_honk(tod_msgs::SecondaryControlCmd &out, const std::vector<int> &buttonState);
};
