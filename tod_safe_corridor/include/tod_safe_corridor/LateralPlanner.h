// Copyright 2020 Simon Hoffmann
#pragma once
#include <iostream>
#include <memory>
#include <string>
#include "geometry_msgs/Point.h"
#include "tod_msgs/Trajectory.h"
#include "tod_core/VehicleParameters.h"
#include "tod_msgs/PrimaryControlCmd.h"

class LateralPlanner {
    public:
        LateralPlanner(std::shared_ptr<tod_core::VehicleParameters> vehParams, float segmentLength);
        virtual ~LateralPlanner() = default;
        LateralPlanner(LateralPlanner&&) = default;
        LateralPlanner(const LateralPlanner&) = default;
        LateralPlanner& operator=(LateralPlanner&&) = default;
        LateralPlanner& operator=(const LateralPlanner&) = default;

        virtual void update_pose(const double x, const double y, const double yaw) = 0;
        virtual tod_msgs::Trajectory calculate_path(const tod_msgs::PrimaryControlCmd& cmd,
            const std::string& frame_id, const std::string& child_frame_id, const double length) = 0;

    protected:
        std::shared_ptr<tod_core::VehicleParameters> _vehParams;
        float _segmentLength{0.3};
};
