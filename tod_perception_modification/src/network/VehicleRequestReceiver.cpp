#include "tod_network/tod_receiver.h"
#include <tod_perception_modification/PercModOperatorRequest.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "VehicleRequestReceiver");
    ros::NodeHandle n;
    tod_network::Receiver<tod_perception_modification::PercModOperatorRequest>
            receiver(n);
    receiver.add_processer("operator_request", tod_network::VehiclePorts::RX_PERCMOD_REQUEST);
    receiver.run();
    return 0;
}
