// Copyright 2021 Feiler
#pragma once
#include "ros/ros.h"
#include <map>
#include <string>
#include <vector>
#include "std_msgs/Bool.h"
#include "sensor_msgs/PointCloud.h"
#include "tod_perception_modification/PercModOperatorRequest.h"
#include "tod_perception_modification/PercModVehicleResponse.h"
#include "tod_perception_modification/PercModOperatorApproval.h"
#include "tod_msgs/Status.h"

class VehicleCommunicator {
public:
    VehicleCommunicator();
    void publish();

private:
    ros::NodeHandle nodeHandle;
    std::vector<ros::Subscriber> subscribers;
    std::map<std::string, ros::Publisher> publishers;

    tod_perception_modification::PercModOperatorRequest operatorRequest;
    tod_perception_modification::PercModVehicleResponse vehicleResponse;
    tod_perception_modification::PercModOperatorApproval operatorApproval;

    bool todMode;
    bool gotNewToDMode {true};
    bool markedAreaIsApproved {false};

    void initToDMode();
    void publishToDMode();
    void tellPurePursuitToWork(const bool tod_mode);
    void publishTodModeIfNew();
    void tellVehiclePercNodeToDeletePreviousArea();
    void createVehicleResponse();
    void publishVehicleResponseToBeSentToOperator();
    void publishApplyModification(const bool applyModification);
    void publishArea();
    void answerOperatorRequest();

	void callbackInputTodMode(const std_msgs::Bool& msg);
    void callbackOperatorRequest(
            const tod_perception_modification::PercModOperatorRequest& msg);
    void callbackOperatorApproval(
            const tod_perception_modification::PercModOperatorApproval& msg);
    void publishApplyAndAreaForPercModNode();
    void callbackOperatorIncrementVelocity(
            const std_msgs::Bool& incrementPositively);
    void callbackStatusMsg(const tod_msgs::Status& msg);
    void switchFromAVToTodMode(const tod_msgs::Status& msg);
    void deleteAreaWhenNotInTeleoperationAnymore(
		const tod_msgs::Status& msg);
};
