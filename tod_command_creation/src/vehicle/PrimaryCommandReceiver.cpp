// Copyright 2020 Simon Hoffmann
#include "tod_network/tod_receiver.h"
#include "tod_msgs/PrimaryControlCmd.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "PrimaryCommandReceiver");
    ros::NodeHandle n;
    tod_network::Receiver<tod_msgs::PrimaryControlCmd>
            receiver(n);
    receiver.add_processer("primary_control_cmd", tod_network::VehiclePorts::RX_PRIMARYCONTROL_COMMAND);
    receiver.run();
    return 0;
}
