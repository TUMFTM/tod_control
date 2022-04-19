// Copyright 2020 Simon Hoffmann
#include "tod_network/tod_sender.h"
#include "tod_safety_monitoring/GateState.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "GateStateSender");
    ros::NodeHandle n;
    tod_network::Sender<tod_safety_monitoring::GateState> sender(n,
        tod_network::OperatorPorts::RX_GATE_STATE, false);
    sender.run();
    return 0;
}
