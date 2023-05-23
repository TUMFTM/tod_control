#include "tod_network/tod_receiver.h"
#include <tod_msgs/PrimaryControlCmd.h>
#include <tod_msgs/ObjectList.h>
#include <tod_msgs/ColoredPolygon.h>
#include <tod_shared_control/MpcLog.h>
#include <tod_shared_control/SvcLog.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "SharedControlReceiver");
    ros::NodeHandle n;
    using namespace tod_network;
    using tod_network::OperatorPorts;

    Receiver<tod_msgs::PrimaryControlCmd> cmdRecv(n);
    cmdRecv.add_processer("primary_control_cmd", RX_SHAREDCONTROL_COMMAND);
    cmdRecv.receive();

    Receiver<tod_msgs::PrimaryControlCmd> shadowCmdRecv(n);
    shadowCmdRecv.add_processer("Shadow/primary_control_cmd", RX_SHAREDCONTROL_SHADOW_COMMAND);
    shadowCmdRecv.receive();

    Receiver<tod_msgs::ObjectList> objectsRecv(n);
    objectsRecv.add_processer("avoided_obstacles", RX_SHAREDCONTROL_OBJECTS);
    objectsRecv.receive();

    Receiver<tod_shared_control::MpcLog> mpcLogRecv(n);
    mpcLogRecv.add_processer("mpc_log", RX_SHAREDCONTROL_MPC_LOG);
    mpcLogRecv.receive();

    Receiver<tod_shared_control::SvcLog> svcLogRecv(n);
    svcLogRecv.add_processer("svc_log", RX_SHAREDCONTROL_SVC_LOG);
    svcLogRecv.receive();

    Receiver<tod_msgs::ColoredPolygon> polygonRecv(n);
    polygonRecv.add_processer("predicted_polygon", RX_SHAREDCONTROL_POLYGON);
    polygonRecv.receive();

    ros::spin();

    return 0;
}
