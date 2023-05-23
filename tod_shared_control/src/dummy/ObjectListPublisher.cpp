// Copyright 2021 Schimpe
#include <ros/ros.h>
#include <tod_msgs/ObjectList.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "DummyObjectListPublisher");
    ros::NodeHandle nh;
    std::string nodeName = ros::this_node::getName();
    std::string topic{"/Vehicle/Lidar/dummy_object_list"};
    ros::Publisher pub = nh.advertise<tod_msgs::ObjectList>(topic, 5);

    tod_msgs::ObjectList msg;
    msg.header.frame_id = "ftm";
    msg.header.stamp = ros::Time::now();

#define RANDOM_OBSTACLE_FIELD 0
#if RANDOM_OBSTACLE_FIELD
    for (int i=0; i < 500; ++i) {
        auto &obj1 = msg.objectList.emplace_back();
        obj1.distCenterX = double(std::rand()) / RAND_MAX * 500.0 + 10.0;
        obj1.distCenterY = double(std::rand()) / RAND_MAX * 500.0 - 250.0;
        obj1.dimX = obj1.dimY = obj1.dimZ = 1.0f;
    }
#else
    std::vector<double> objDistsY{6.0, -4.0, 3.0, -2.0, 0.0};
    const double dx = 17.5;
    double prevX = 0.0;
    for (const auto distY : objDistsY) {
        auto &obj1 = msg.objectList.emplace_back();
        prevX = obj1.distCenterX = prevX + dx;
        obj1.distCenterY = distY;
        obj1.dimX = obj1.dimY = 2.5f;
        obj1.dimZ = 1.0f;
    }
    msg.objectList.back().distCenterX += dx;
    msg.objectList.back().dimX *= 2.0;
    msg.objectList.back().dimY *= 2.0;
#endif

    ros::Rate r(10);
    while (ros::ok()) {
        ros::spinOnce();
        pub.publish(msg);
        r.sleep();
    }

    return 0;
}
