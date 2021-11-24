#include <ros/ros.h>
#include <tod_msgs/Status.h>
#include <tod_network/udp_receiver.h>
#include <tod_msgs/PaketInfo.h>
#include <memory>

template <typename T>
class ReceiverOfMsgWithoutHeader {
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
    ReceiverOfMsgWithoutHeader(ros::NodeHandle &n, int port) : _n(n) { add_processer("/received_topic", port); }
    explicit ReceiverOfMsgWithoutHeader(ros::NodeHandle &n) : _n(n) {}

    void add_processer(const std::string &topic, const int port) {
        auto receiver = _rosMsgReceivers.emplace_back(std::make_shared<RosMsgReceiver>());
        receiver->udpReceiver = std::make_unique<tod_network::UdpReceiver>(port);
        receiver->recvMsgPubs = _n.advertise<T>(topic, 1);
        receiver->recvMsgPaketInfoPubs = _n.advertise<tod_msgs::PaketInfo>(topic + "_paket_info", 1);
    }

    void receive() {
        for (auto receiver : _rosMsgReceivers)
            receiver->thread = std::make_unique<std::thread>
                        (&ReceiverOfMsgWithoutHeader::receive_msgs, this, receiver);
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

#include "std_msgs/Bool.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "VehicleVelocityReceiver");
    ros::NodeHandle n;
    ReceiverOfMsgWithoutHeader<std_msgs::Bool>receiver(n);
    receiver.add_processer("operator_increment_velocity", tod_network::VehiclePorts::RX_PERCMOD_VELOCITY_CHANGE);
    receiver.run();
    return 0;
}
