#include "tod_network/tod_sender.h"
#include <tod_perception_modification/PercModVehicleResponse.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "VehicleResponseSender");
    ros::NodeHandle n;
    tod_network::Sender<tod_perception_modification::PercModVehicleResponse> sender(n, true);
    std::string nodeName = ros::this_node::getName();
    sender.add_processer("/Vehicle/PerceptionModification/vehicle_response",
                         tod_network::OperatorPorts::RX_PERCMOD_RESPONSE);
    sender.run();
    return 0;
}
