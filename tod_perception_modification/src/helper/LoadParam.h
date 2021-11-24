// Copyright 2021 Feiler

#include "ros/ros.h"
#include <string>

namespace TodHelper {
    void printStandardRosWarn(const std::string& paramName,
    const std::string& nodeName) {
        ROS_WARN("could not find %s on param server at %s. Using default value.",
            paramName.c_str(), nodeName.c_str());
    }

    template <class T>
    void loadParam(ros::NodeHandle& nodeHandle, const std::string& paramName,
    T& parameter, const T defaultValue) {
        std::string nodeName = ros::this_node::getName();
        if ( !nodeHandle.getParam(nodeName + "/" + paramName, parameter) ) {
            printStandardRosWarn(paramName, nodeName);
            parameter = defaultValue;
        }
    }

}; //namespace TodHelper
