// Copyright 2021 Hoffmann

#include <gtest/gtest.h>
#include "ros/ros.h"
#include <iostream>
#include "tod_pure_pursuit/PurePursuit.h"
#include "tod_helper/geometry/Helpers.h"

double spacingBetweenPointsInM { 0.3 };
int numberOfTrajPoints { 30 };

tod_msgs::Trajectory initTrajectory() {
    tod_msgs::Trajectory straightTrajectory;
    for ( int trajPointIt = 0; trajPointIt != numberOfTrajPoints; ++trajPointIt ) {
        tod_msgs::TrajectoryPoint tmpPoint;
        tmpPoint.pose.pose.position.x = (double) trajPointIt * spacingBetweenPointsInM;
        straightTrajectory.points.push_back(tmpPoint);
    }
    return straightTrajectory;
}

geometry_msgs::Pose initPose(double x) {
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.orientation.w = 1.0;
    return pose;
}

class TestSuite : public ::testing::Test {
    public:
        TestSuite() {}
        ~TestSuite() {}
        ros::NodeHandle* node;
        PurePursuit* purePursuit;
        tod_msgs::Trajectory trajectory;
        geometry_msgs::PoseStamped pose;

        void SetUp() override {
            ::testing::Test::SetUp();
            node = new ros::NodeHandle();
            purePursuit = new PurePursuit(*node);
            trajectory = initTrajectory();
            pose.pose = initPose(0.0);
        }
        void TearDown() override {
            ros::shutdown();
            delete this->purePursuit;
            delete this->node;
            ::testing::Test::TearDown();
        }
};

TEST_F(TestSuite, helpers) {
    geometry_msgs::Point pt;
    pt.x = 4;
    pt.y = 3;
    ASSERT_EQ(tod_helper::Geometry::calc_horizontal_distance(pt, pose.pose.position), 5);
    ASSERT_EQ(tod_helper::Geometry::calc_relative_position(pt, pose.pose).y, 3);
}

TEST_F(TestSuite, calc_curvature) {
    geometry_msgs::Point pt;
    pt.x = 4;
    pt.y = 3;
    ASSERT_FLOAT_EQ(purePursuit->calc_curvature(pt, pose), 6.0/25.0);
    pt.y = -3; // negative curvature
    ASSERT_FLOAT_EQ(purePursuit->calc_curvature(pt, pose), -6.0/25.0);
    pt.y = 0; // Target straight ahead
    ASSERT_FLOAT_EQ(purePursuit->calc_curvature(pt, pose), 0.0);
    pt.y = 0, pt.x = 0; // Target equals vehicle position
    ASSERT_FLOAT_EQ(purePursuit->calc_curvature(pt, pose), 0.0);
}

TEST_F(TestSuite, find_next_point) {
    // Copying no problem for testing
    tod_msgs::TrajectoryConstPtr traj(new tod_msgs::Trajectory(trajectory));
    ASSERT_EQ(purePursuit->find_next_point(traj, pose), 1);
    pose.pose.position.x = -0.01;
    ASSERT_EQ(purePursuit->find_next_point(traj, pose), 0);
    pose.pose.position.x = 0.60;
    ASSERT_EQ(purePursuit->find_next_point(traj, pose), 3);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tod_pure_pursuit_test");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
