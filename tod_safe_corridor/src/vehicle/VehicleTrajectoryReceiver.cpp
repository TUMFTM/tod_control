// Copyright 2020 Simon Hoffmann
#include "tod_network/tod_receiver.h"
#include "tod_msgs/Trajectory.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "TrajectoryReceiver");
    ros::NodeHandle n;
    tod_network::Receiver<tod_msgs::Trajectory>
            receiver(n, tod_network::VehiclePorts::RX_SAFECORRIDORCONTROL_COMMAND);
    receiver.run();
    return 0;
}
