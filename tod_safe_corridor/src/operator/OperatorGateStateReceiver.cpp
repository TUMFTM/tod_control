// Copyright 2020 Simon Hoffmann
#include "tod_network/tod_receiver.h"
#include "tod_safety_monitoring/GateState.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "GateStateReceiver");
    ros::NodeHandle n;
    tod_network::Receiver<tod_safety_monitoring::GateState>
            receiver(n, tod_network::OperatorPorts::RX_GATE_STATE);
    receiver.run();
    return 0;
}
