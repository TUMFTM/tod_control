#include "tod_network/tod_sender.h"
#include <tod_perception_modification/PercModOperatorApproval.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "OperatorRequestSender");
    ros::NodeHandle n;
    tod_network::Sender<tod_perception_modification::PercModOperatorApproval> sender(n, false);
    sender.add_processer("operator_approval", tod_network::VehiclePorts::RX_PERCMOD_APPROVAL);
    sender.run();
    return 0;
}
