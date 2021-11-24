#include "tod_network/tod_receiver.h"
#include <tod_perception_modification/PercModVehicleResponse.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "OperatorResponseReceiver");
    ros::NodeHandle n;
    tod_network::Receiver<tod_perception_modification::PercModVehicleResponse> receiver(n);
    std::string nodeName = ros::this_node::getName();
    receiver.add_processer("/Operator/PerceptionModification/vehicle_response",
                           tod_network::OperatorPorts::RX_PERCMOD_RESPONSE);
    receiver.run();
    return 0;
}
