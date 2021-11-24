// Copyright 2021 Feiler
#pragma once
#include "ros/ros.h"
#include "tod_msgs/PrimaryControlCmd.h"
#include <vector>
#include <map>
#include <string>
#include "std_msgs/Bool.h"

class ControlCommandCreator {
public:
    ControlCommandCreator();
    void publish();

private:
    ros::NodeHandle nodeHandle;
    std::vector<ros::Subscriber> subscribers;
    std::map<std::string, ros::Publisher> publishers;

    tod_msgs::PrimaryControlCmd primaryControl;
    bool todMode {true};
    bool modeSwitchedFromAVtoTod {false};

    void callbackPrimaryControlCmd(const tod_msgs::PrimaryControlCmdConstPtr msg);
    void callbackTodMode(const std_msgs::Bool& tod_mode);
    void publishControlCommands();
    void publishStandStillControlCommand();
};
