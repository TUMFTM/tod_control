#include "tod_network/tod_sender.h"
#include "grid_map_msgs/GridMap.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "VehicleGridMapSender");
    ros::NodeHandle n;
    tod_network::Sender<grid_map_msgs::GridMap> sender(n, true);
    std::string nodeName = ros::this_node::getName();
    sender.add_processer("/Vehicle/PerceptionModification/appended_grid_map",
                         tod_network::OperatorPorts::RX_PERCMOD_GRIDMAP);
    sender.run();
    return 0;
}
