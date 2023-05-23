#include "tod_network/tod_sender.h"
#include <tod_msgs/PrimaryControlCmd.h>
#include <tod_msgs/ObjectList.h>
#include <tod_msgs/ColoredPolygon.h>
#include <tod_shared_control/MpcLog.h>
#include <tod_shared_control/SvcLog.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "SharedControlSender");
    ros::NodeHandle n;
    using namespace tod_network;
    using tod_network::OperatorPorts;

    Sender<tod_msgs::PrimaryControlCmd> sharedCtrlCmdSender(n, true);
    sharedCtrlCmdSender.add_processer("primary_control_cmd", RX_SHAREDCONTROL_COMMAND);

    Sender<tod_msgs::PrimaryControlCmd> shadowSharedCtrlCmdSender(n, true);
    shadowSharedCtrlCmdSender.add_processer("Shadow/primary_control_cmd", RX_SHAREDCONTROL_SHADOW_COMMAND);

    Sender<tod_msgs::ObjectList> objectListSender(n, true);
    objectListSender.add_processer("avoided_obstacles", RX_SHAREDCONTROL_OBJECTS);

    Sender<tod_shared_control::MpcLog> mpcLogSender(n, true);
    mpcLogSender.add_processer("mpc_log", RX_SHAREDCONTROL_MPC_LOG);

    Sender<tod_shared_control::SvcLog> svcLogSender(n, true);
    svcLogSender.add_processer("svc_log", RX_SHAREDCONTROL_SVC_LOG);

    Sender<tod_msgs::ColoredPolygon> polygonSender(n, true);
    polygonSender.add_processer("predicted_polygon", RX_SHAREDCONTROL_POLYGON);

    ros::MultiThreadedSpinner spinner{6};
    spinner.spin();

    return 0;
}
