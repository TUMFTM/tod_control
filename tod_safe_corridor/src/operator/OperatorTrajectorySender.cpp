// Copyright 2020 Simon Hoffmann
#include "tod_network/tod_sender.h"
#include "tod_msgs/Trajectory.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "TrajectorySender");
    ros::NodeHandle n;
    tod_network::Sender<tod_msgs::Trajectory> sender(n,
        tod_network::VehiclePorts::RX_SAFECORRIDORCONTROL_COMMAND, false);
    sender.run();
    return 0;
}
