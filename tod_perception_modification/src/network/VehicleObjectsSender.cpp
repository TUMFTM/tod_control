#include "tod_network/tod_sender.h"
#include "tod_msgs/ObjectList.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "VehicleObjectsSender");
    ros::NodeHandle n;
    tod_network::Sender<tod_msgs::ObjectList> sender(n, true);
    std::string nodeName = ros::this_node::getName();
    sender.add_processer("/Vehicle/PerceptionModification/appended_objects",
                         tod_network::OperatorPorts::RX_PERCMOD_OBJECTS);
    sender.run();
    return 0;
}
