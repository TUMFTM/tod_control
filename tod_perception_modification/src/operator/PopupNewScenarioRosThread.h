//Copyright 2021 Feiler

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <mutex>

class PopupNewScenarioRosThread {

public:
    void runRosLoop() {
        ros::NodeHandle nodeHandle;
        ros::Rate rate(30);
        ros::Publisher pubTaskNumber = nodeHandle.advertise<std_msgs::Int32>(
            "task_number", 4);
        while ( ros::ok() ) {
            ros::spinOnce();
            if ( gotNewTaskNumber ) {
                gotNewTaskNumber = false;
                std_msgs::Int32 msg;
                msg.data = taskNumber;
                pubTaskNumber.publish(msg);
            }
            rate.sleep();
        }
        loopIsLeft = true;
    }

    bool rosWasShutDown() {
        return loopIsLeft;
    }

    void publishThisTaskNumber(int number) {
        taskNumber = number;
        gotNewTaskNumber = true;
    }

private:
    ros::NodeHandle nodeHandle;
    bool loopIsLeft { false };

    int taskNumber;
    bool gotNewTaskNumber;
};
