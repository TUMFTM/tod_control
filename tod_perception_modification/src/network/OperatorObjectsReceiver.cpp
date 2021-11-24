#include "tod_network/tod_receiver.h"
#include "tod_msgs/ObjectList.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "OperatorObjectsReceiver");
    ros::NodeHandle n;
    tod_network::Receiver<tod_msgs::ObjectList> receiver(n);
    std::string nodeName = ros::this_node::getName();
    receiver.add_processer("/Operator/PerceptionModification/appended_objects",
                           tod_network::OperatorPorts::RX_PERCMOD_OBJECTS);
    receiver.run();
    return 0;
}
