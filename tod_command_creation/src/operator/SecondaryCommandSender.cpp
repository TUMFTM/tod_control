// Copyright 2020 Simon Hoffmann
#include "tod_network/tod_sender.h"
#include "tod_msgs/SecondaryControlCmd.h"
int main(int argc, char **argv) {
    ros::init(argc, argv, "SecondaryCommandSender");
    ros::NodeHandle n;
    tod_network::Sender<tod_msgs::SecondaryControlCmd> sender(n, false);
    sender.add_processer("secondary_control_cmd", tod_network::VehiclePorts::RX_SECONDARY_COMMAND);
    sender.run();
    return 0;
}
