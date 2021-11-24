// Copyright 2021 Feiler

#include "VehicleCommunicator.h"
#include "tod_msgs/Status.h"
#include "LoadParam.h"
#include <utility>

VehicleCommunicator::VehicleCommunicator() {
    publishers.insert(std::pair<std::string, ros::Publisher>(
        "PubStatusForPurePursuit", nodeHandle.advertise<tod_msgs::Status>(
        "perception_modification_status_for_pure_pursuit", 2)));

    publishers.insert(std::pair<std::string, ros::Publisher>(
        "Pubtod_mode", nodeHandle.advertise<std_msgs::Bool>(
        "tod_mode", 2, true)));
    subscribers.push_back(nodeHandle.subscribe("input_tod_mode", 5,
                    &VehicleCommunicator::callbackInputTodMode, this));

    subscribers.push_back(nodeHandle.subscribe("operator_request", 5,
                    &VehicleCommunicator::callbackOperatorRequest, this));
    publishers.insert(std::pair<std::string, ros::Publisher>(
        "vehicle_response", nodeHandle.advertise
        <tod_perception_modification::PercModVehicleResponse>(
        "vehicle_response", 2)));
    subscribers.push_back(nodeHandle.subscribe("operator_approval", 5,
                    &VehicleCommunicator::callbackOperatorApproval, this));

    publishers.insert(std::pair<std::string, ros::Publisher>(
        "apply_modification", nodeHandle.advertise
        <std_msgs::Bool>("apply_modification", 2)));
    publishers.insert(std::pair<std::string, ros::Publisher>(
        "area", nodeHandle.advertise<sensor_msgs::PointCloud>("area", 2)));

    subscribers.push_back(nodeHandle.subscribe("operator_increment_velocity", 5,
                    &VehicleCommunicator::callbackOperatorIncrementVelocity, this));
    publishers.insert(std::pair<std::string, ros::Publisher>(
        "increment_velocity", nodeHandle.advertise
        <std_msgs::Bool>("increment_velocity", 2)));

    subscribers.push_back(nodeHandle.subscribe("status_msg", 5,
                    &VehicleCommunicator::callbackStatusMsg, this));
    initToDMode();
}

void VehicleCommunicator::callbackInputTodMode(const std_msgs::Bool& msg) {
    todMode = msg.data;
    gotNewToDMode = true;
    ROS_WARN("got a manual input to change todMode at node %s", ros::this_node::getName().c_str());
}

void VehicleCommunicator::callbackOperatorRequest(
const tod_perception_modification::PercModOperatorRequest& msg) {
    markedAreaIsApproved = false;
    operatorRequest = msg;
    if ( operatorRequest.mouseClicks.points.empty() ) {
        tellVehiclePercNodeToDeletePreviousArea();
    } else {
        answerOperatorRequest();
    }
}

void VehicleCommunicator::tellVehiclePercNodeToDeletePreviousArea() {
    publishApplyModification(false);
    publishArea();
}

void VehicleCommunicator::publishApplyModification(const bool applyModification) {
    std_msgs::Bool applyModificationMsg;
    applyModificationMsg.data = applyModification;
    publishers["apply_modification"].publish(applyModificationMsg);
}

void VehicleCommunicator::publishArea() {
    sensor_msgs::PointCloud area;
    area = operatorRequest.mouseClicks;
    publishers["area"].publish(area);
}

void VehicleCommunicator::answerOperatorRequest() {
    createVehicleResponse();
    publishVehicleResponseToBeSentToOperator();
}

void VehicleCommunicator::createVehicleResponse() {
    vehicleResponse.header.stamp = ros::Time::now();
    vehicleResponse.RequestId = operatorRequest.RequestId;
    vehicleResponse.mouseClicksAreAccepted = true;
    ROS_ERROR("at vehicle side, I received an operator request and I am answering");
}

void VehicleCommunicator::publishVehicleResponseToBeSentToOperator() {
    publishers["vehicle_response"].publish(vehicleResponse);
}

void VehicleCommunicator::callbackOperatorApproval(
const tod_perception_modification::PercModOperatorApproval& msg) {
    operatorApproval = msg;
    ROS_ERROR("at vehicle side - approval arrived");
    if ( operatorApproval.RequestId == vehicleResponse.RequestId &&
    operatorApproval.operatorApproved) {
        markedAreaIsApproved = true;
        publishApplyAndAreaForPercModNode();
    }
}

void VehicleCommunicator::publishApplyAndAreaForPercModNode() {
    publishApplyModification(true);
    publishArea();
}

void VehicleCommunicator::callbackOperatorIncrementVelocity(
const std_msgs::Bool& incrementPositively) {
    if ( markedAreaIsApproved ) {
        publishers["increment_velocity"].publish(incrementPositively);
    }
}

void VehicleCommunicator::initToDMode() {
    gotNewToDMode = true;
    todMode = true;
    bool startInADMode;
    TodHelper::loadParam(nodeHandle, "startInADMode", startInADMode, false);
    if ( startInADMode == true ) {
        todMode = false;
    }
}

void VehicleCommunicator::publish() {
    tellPurePursuitToWork(todMode);
    publishTodModeIfNew();
}

void VehicleCommunicator::tellPurePursuitToWork(const bool tod_mode) {
    tod_msgs::Status status;
    if ( tod_mode ) {
        status.vehicle_control_mode =
            tod_msgs::Status::CONTROL_MODE_NONE;
    } else { // AV mode
        status.vehicle_control_mode =
            tod_msgs::Status::CONTROL_MODE_PERCEPTION_MODIFICATION;
    }
    publishers["PubStatusForPurePursuit"].publish(status);
}

void VehicleCommunicator::publishTodModeIfNew() {
    if ( gotNewToDMode ) {
        gotNewToDMode = false;
        publishToDMode();
    }
}

void VehicleCommunicator::publishToDMode() {
    std_msgs::Bool tod_mode;
    tod_mode.data = todMode;
    publishers["Pubtod_mode"].publish(tod_mode);
}

void VehicleCommunicator::callbackStatusMsg(const tod_msgs::Status& msg) {
    switchFromAVToTodMode(msg);
    deleteAreaWhenNotInTeleoperationAnymore(msg);
}

void VehicleCommunicator::switchFromAVToTodMode(const tod_msgs::Status& msg) {
    static bool onlyOnceAfterChangeIntoToDMode = true;
    if ( msg.tod_status == tod_msgs::Status::TOD_STATUS_TELEOPERATION &&
    msg.operator_control_mode != tod_msgs::Status::CONTROL_MODE_PERCEPTION_MODIFICATION ) {
        if ( onlyOnceAfterChangeIntoToDMode ) {
            onlyOnceAfterChangeIntoToDMode = false;
            todMode = true;
            gotNewToDMode = true;
        }
    } else {
        onlyOnceAfterChangeIntoToDMode = true;
    }
}

void VehicleCommunicator::deleteAreaWhenNotInTeleoperationAnymore(
const tod_msgs::Status& msg) {
    static bool onlyOnceWhenNotInTeleoperationAnymore = true;
    if ( onlyOnceWhenNotInTeleoperationAnymore && msg.tod_status !=
    tod_msgs::Status::TOD_STATUS_TELEOPERATION ) {
        onlyOnceWhenNotInTeleoperationAnymore = false;
        tellVehiclePercNodeToDeletePreviousArea();
    } else {
        onlyOnceWhenNotInTeleoperationAnymore = true;
    }
}
