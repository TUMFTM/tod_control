// Copyright 2021 Simon Hoffmann
#pragma once
#include "tod_msgs/Trajectory.h"

class SpeedPlannerParams{
public:
    SpeedPlannerParams() = default;
    explicit SpeedPlannerParams(ros::NodeHandle& nh) {
        load_from_parameter_workspace(nh);
    }
    void load_from_parameter_workspace(ros::NodeHandle& nh) {
        if (!nh.getParam(ros::this_node::getName() + "/desiredDeceleration", desiredDeceleration))
            ROS_ERROR("%s: Could not get parameter /desiredDeceleration - using %f",
                    ros::this_node::getName().c_str(), desiredDeceleration);
        if (!nh.getParam(ros::this_node::getName() + "/timeOffsetToDeceleration", timeOffsetToDeceleration))
            ROS_ERROR("%s: Could not get parameter /timeOffsetToDeceleration - using %f",
                    ros::this_node::getName().c_str(), timeOffsetToDeceleration);
    }

    double desiredDeceleration{4.0};
    double timeOffsetToDeceleration{0.2};
};

class LongitudinalPlanner {
    public:
        LongitudinalPlanner() = default;
        virtual ~LongitudinalPlanner() = default;
        LongitudinalPlanner(LongitudinalPlanner&&) = default;
        LongitudinalPlanner& operator=(LongitudinalPlanner&&) = default;
        LongitudinalPlanner(const LongitudinalPlanner&) = default;
        LongitudinalPlanner& operator=(const LongitudinalPlanner&) = default;

        virtual void apply_velocity_profile(tod_msgs::Trajectory& traj, const double velocity, const double offset,
            const double deceleration) = 0;
        virtual double get_braking_distance(const double velocity, const double offset,
            const double deceleration) = 0;
        void set_stamp_of_ref_point(const ros::Time& time) {
            _refTime = time;
            _TimeIsSetManually = true;
        }
    protected:
        ros::Time get_stamp_of_ref_point() {return _TimeIsSetManually ? _refTime : ros::Time::now();}
    private:
        ros::Time _refTime;
        bool _TimeIsSetManually{false};
};
