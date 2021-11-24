// Copyright 2021 Feiler

#include "ros/ros.h"
#include "tod_network/mqtt_client.h"
#include "tod_msgs/Status.h"
#include <mutex>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

std::unique_ptr<tod_network::MqttClient> mqttClientToOperator;
bool mqttClientToOperatorCreated { false };
std::string topicName { "/TaskTrigger" };

void rosCallbackIpAddress(const std_msgs::String& msg);
std::string ipAddressOperator;

void rosCallbackTaskTrigger(const std_msgs::Int32& msg);

void createMqttClient();

int main(int argc, char **argv) {
    ros::init(argc, argv, "VehiclePopupTrigger");
    ros::NodeHandle nodeHandle;
    ros::Subscriber subToOperatorIpAddress =
        nodeHandle.subscribe("sub_ip_address_operator", 5, rosCallbackIpAddress);
    ros::Subscriber subToTriggerTask =
        nodeHandle.subscribe("sub_trigger_task_number", 5, rosCallbackTaskTrigger);
    ros::spin();
    return 0;
}

void rosCallbackIpAddress(const std_msgs::String& msg) {
    ipAddressOperator = msg.data;
    if ( !mqttClientToOperatorCreated ) {
        createMqttClient();
    }
}

void rosCallbackTaskTrigger(const std_msgs::Int32& msg) {
    if ( mqttClientToOperator->is_connected() ) {
        ros::SerializedMessage rosSer =
            ros::serialization::serializeMessage<std_msgs::Int32>(msg);
        mqttClientToOperator->publish(topicName, 1,
            (char*) rosSer.message_start, rosSer.num_bytes);

    }
}

void createMqttClient() {
    mqttClientToOperator = std::make_unique<tod_network::MqttClient>(ipAddressOperator,
        "VehiclePopupTrigger");
    if ( !mqttClientToOperator->is_connected() ) {
        ROS_ERROR("Could not connect to operator broker. Connected?");
    } else {
        printf("Successfully connected to operator\n");
        mqttClientToOperatorCreated = true;
    }
}

