#include "tod_network/tod_sender.h"
#include "std_msgs/Bool.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "OperatorVelocitySender");
    ros::NodeHandle n;
    tod_network::Sender<std_msgs::Bool> sender(n, false);
    sender.add_processer("operator_increment_velocity", tod_network::VehiclePorts::RX_PERCMOD_VELOCITY_CHANGE);
    sender.run();
    return 0;
}
