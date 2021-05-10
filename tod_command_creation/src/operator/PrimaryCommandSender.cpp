// Copyright 2020 Simon Hoffmann
#include "tod_network/tod_sender.h"
#include "tod_msgs/PrimaryControlCmd.h"
int main(int argc, char **argv) {
    ros::init(argc, argv, "PrimaryCommandSender");
    ros::NodeHandle n;
    tod_network::Sender<tod_msgs::PrimaryControlCmd> sender(n, false);
    sender.add_processer("primary_control_cmd", tod_network::VehiclePorts::RX_PRIMARYCONTROL_COMMAND);
    sender.send_in_control_mode(tod_msgs::Status::CONTROL_MODE_DIRECT);
    sender.send_in_control_mode(tod_msgs::Status::CONTROL_MODE_SHARED);
    sender.run();
    return 0;
}
