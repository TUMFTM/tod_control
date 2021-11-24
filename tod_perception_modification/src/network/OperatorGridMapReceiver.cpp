#include <ros/ros.h>
#include <tod_msgs/Status.h>
#include <tod_network/udp_receiver.h>
#include <tod_msgs/PaketInfo.h>
#include <memory>

template <typename T>
class Receiver {
private:
    struct RosMsgReceiver{
        ros::Publisher recvMsgPubs, recvMsgPaketInfoPubs;
        std::unique_ptr<tod_network::UdpReceiver> udpReceiver{nullptr};
        bool printedInfo{false};
        std::unique_ptr<std::thread> thread{nullptr};
        T msg;
        tod_msgs::PaketInfo paketInfoMsg;
    };

public:
    Receiver(ros::NodeHandle &n, int port) : _n(n) { add_processer("/received_topic", port); }
    explicit Receiver(ros::NodeHandle &n) : _n(n) {}

    void add_processer(const std::string &topic, const int port) {
        auto receiver = _rosMsgReceivers.emplace_back(std::make_shared<RosMsgReceiver>());
        receiver->udpReceiver = std::make_unique<tod_network::UdpReceiver>(port);
        receiver->recvMsgPubs = _n.advertise<T>(topic, 1);
        receiver->recvMsgPaketInfoPubs = _n.advertise<tod_msgs::PaketInfo>(topic + "_paket_info", 1);
    }

    void receive() {
        for (auto receiver : _rosMsgReceivers)
            receiver->thread = std::make_unique<std::thread>(&Receiver::receive_msgs, this, receiver);
    }

    void run() {
        receive();
        ros::spin();
    }


private:
    ros::NodeHandle &_n;
    std::vector<std::shared_ptr<RosMsgReceiver>> _rosMsgReceivers;

    void receive_msgs(std::shared_ptr<RosMsgReceiver> receiver) {
        while (ros::ok()) {
            int sizeByte = receiver->udpReceiver->receive_ros_msg(receiver->msg);

            // publish paket info msg
            receiver->paketInfoMsg.header.stamp = ros::Time::now();
//            receiver->paketInfoMsg.seqNum = receiver->msg.info.header.seq;
            receiver->paketInfoMsg.sizeBit = sizeByte * 8;
//            receiver->paketInfoMsg.latencyUsec = int64_t(ros::Time::now().toNSec()/1000
//                                                         - receiver->msg.info.header.stamp.toNSec()/1000);
            receiver->recvMsgPaketInfoPubs.publish(receiver->paketInfoMsg);

            // publish restamped data msg
//            receiver->msg.info.header.stamp = ros::Time::now();
            receiver->recvMsgPubs.publish(receiver->msg);
            if (!receiver->printedInfo) {
                receiver->printedInfo = true;
                ROS_INFO("%s: receiving topic %s", ros::this_node::getName().c_str(),
                         receiver->recvMsgPubs.getTopic().c_str());
            }
        }
    }
};

#include "grid_map_msgs/GridMap.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "OperatorGridMapReceiver");
    ros::NodeHandle n;
    Receiver<grid_map_msgs::GridMap> receiver(n);
    std::string nodeName = ros::this_node::getName();
    receiver.add_processer("/Operator/PerceptionModification/appended_grid_map",
                           tod_network::OperatorPorts::RX_PERCMOD_GRIDMAP);
    receiver.run();
    return 0;
}
