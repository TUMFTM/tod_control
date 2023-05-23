#include <ros/ros.h>
#include <tod_msgs/Trajectory.h>
#include <tf2/utils.h>

void generate_trajectory(tod_msgs::Trajectory &trajectory);

int main(int argc, char **argv) {
    ros::init(argc, argv, "DummyTrajectoryGenerator");
    ros::NodeHandle nh;
    std::string nodeName = ros::this_node::getName();
    ros::Publisher pubTrajectory = nh.advertise<tod_msgs::Trajectory>("trajectory", 1);
    tod_msgs::Trajectory traj;
    traj.header.frame_id = "ftm";
    traj.child_frame_id = "rear_axle_footprint";
    generate_trajectory(traj);
    ros::Rate r{10};
    while (ros::ok()) {
        ros::spinOnce();
        traj.header.stamp = ros::Time::now();
        pubTrajectory.publish(traj);
        r.sleep();
    }
    return 0;
}

void generate_trajectory(tod_msgs::Trajectory &trajectory) {
    trajectory.points.clear();
    const float maxVelocity = 5.0f;
    const int length{300};
    const int ptsPerMeter{4};
    for (int x = 0; x < length; ++x) {
        for (int i = 0; i < ptsPerMeter; ++i) {
            auto &pt = trajectory.points.emplace_back(tod_msgs::TrajectoryPoint());
            pt.pose.pose.position.x = float(x) + float(i) / float(ptsPerMeter);
            pt.pose.pose.position.y = 0.0f;
            pt.twist.twist.linear.x = maxVelocity;
            tf2::Quaternion quat(0.0f, 0.0f, 0.0f);
            tf2::convert(quat, pt.pose.pose.orientation);
        }
    }
    const int brakeToZeroVelocityOverIdxs{10 * ptsPerMeter};
    for (int i = 0; i < brakeToZeroVelocityOverIdxs; ++i) {
        int x = trajectory.points.size() - brakeToZeroVelocityOverIdxs + i;
        trajectory.points.at(x).twist.twist.linear.x = maxVelocity - float(i+1) * (maxVelocity / brakeToZeroVelocityOverIdxs);
    }
}
