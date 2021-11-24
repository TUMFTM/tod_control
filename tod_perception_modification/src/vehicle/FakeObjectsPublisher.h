// Copyright 2021 Feiler
#pragma once
#include "ros/ros.h"
#include "FakeThingsPublisherAbstract.h"
#include <memory>
#include <string>
#include <vector>
#include "tod_msgs/ObjectList.h"
#include "Timer.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class FakeObjectsPublisher : public FakeThingsPublisherAbstract {
public:
    FakeObjectsPublisher();
    virtual void initFromParamServer(const std::string& nodeName);
    virtual bool isCreateMode();
    virtual void create();
    virtual void publish();
    void callbackObjectList(const tod_msgs::ObjectListConstPtr msg);

private:
    ros::NodeHandle nodeHandle;
    Mode currentMode;
    std::unique_ptr<Timer> timer;
    tod_msgs::ObjectList fakeObjects;
    tod_msgs::ObjectList objectListToBePublished;
    std::vector<ros::Subscriber> subscribers;
    ros::Publisher pubObjectList;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    std::string desiredObjectsFrameId;

    void lookUpAndAddObject(const int objectNumber, const std::string& nodeName);
    void appendFakeObjectsToList();
    void loadDesiredObjectsFrameId(const std::string& nodeName);
};
