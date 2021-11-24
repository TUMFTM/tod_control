// Copyright 2021 Feiler

#include "ControlCommandCreator.h"
#include <utility>

ControlCommandCreator::ControlCommandCreator() {
    subscribers.push_back(nodeHandle.subscribe<>(
        "primary_control_cmd_input", 5,
        &ControlCommandCreator::callbackPrimaryControlCmd, this));
    subscribers.push_back(nodeHandle.subscribe(
        "tod_mode", 5, &ControlCommandCreator::callbackTodMode, this));
    publishers.insert(std::pair<std::string, ros::Publisher>(
        "PubPrimaryControl", nodeHandle.advertise<tod_msgs::PrimaryControlCmd>(
        "primary_control_cmd", 5)));
}

void ControlCommandCreator::callbackPrimaryControlCmd(
const tod_msgs::PrimaryControlCmdConstPtr msg) {
    primaryControl = *msg;
}

void ControlCommandCreator::callbackTodMode(const std_msgs::Bool& tod_mode) {
    bool previousToDMode = todMode;
    todMode = tod_mode.data;
    if ( todMode == true && previousToDMode != todMode ) {
        modeSwitchedFromAVtoTod = true;
    }
}

void ControlCommandCreator::publish() {
    if ( !todMode ) {
        publishControlCommands();
    }
    if ( modeSwitchedFromAVtoTod ) {
        modeSwitchedFromAVtoTod = false;
        publishStandStillControlCommand();
    }
}

void ControlCommandCreator::publishControlCommands() {
    publishers["PubPrimaryControl"].publish(primaryControl);
}

void ControlCommandCreator::publishStandStillControlCommand() {
    tod_msgs::PrimaryControlCmd standStill;
    standStill.acceleration = 0.0;
    standStill.velocity = 0.0;
    standStill.steeringWheelAngle = primaryControl.steeringWheelAngle;
    publishers["PubPrimaryControl"].publish(standStill);
}
