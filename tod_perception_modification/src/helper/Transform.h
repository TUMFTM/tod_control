// Copyright 2021 Feiler

#include "ros/ros.h"
#include <string>
#include <tf2_ros/buffer.h>
#include "geometry_msgs/TransformStamped.h"

namespace TodHelper {

bool getTransformToTargetFromSource(const std::string& targetFrame,
const std::string& sourceFrame,
geometry_msgs::TransformStamped& transform,
tf2_ros::Buffer& tfBuffer) {
    bool gotTransform { false };
    try {
        transform = tfBuffer.lookupTransform(
            targetFrame, sourceFrame, ros::Time(0));
        gotTransform = true;
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s at %s", ex.what(), ros::this_node::getName().c_str());
    }
    return gotTransform;
}

}; //namespace TodHelper
