// Copyright 2020 Simon Hoffmann
#include "tod_network/tod_receiver.h"
#include "tod_msgs/SecondaryControlCmd.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "SecondaryCommandReceiver");
    ros::NodeHandle n;
    tod_network::Receiver<tod_msgs::SecondaryControlCmd> receiver(n);
    receiver.add_processer("secondary_control_cmd", tod_network::VehiclePorts::RX_SECONDARY_COMMAND);
    receiver.run();
    return 0;
}
