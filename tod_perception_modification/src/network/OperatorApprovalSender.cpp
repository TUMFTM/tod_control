#include "tod_network/tod_sender.h"
#include <tod_perception_modification/PercModOperatorRequest.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "OperatorApprovalSender");
    ros::NodeHandle n;
    tod_network::Sender<tod_perception_modification::PercModOperatorRequest> sender(n, false);
    sender.add_processer("operator_request", tod_network::VehiclePorts::RX_PERCMOD_REQUEST);
    sender.run();
    return 0;
}
