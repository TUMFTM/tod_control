#include "tod_network/tod_receiver.h"
#include <tod_perception_modification/PercModOperatorApproval.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "VehicleRequestReceiver");
    ros::NodeHandle n;
    tod_network::Receiver<tod_perception_modification::PercModOperatorApproval>
            receiver(n);
    receiver.add_processer("operator_approval", tod_network::VehiclePorts::RX_PERCMOD_APPROVAL);
    receiver.run();
    return 0;
}
