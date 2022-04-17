// Copyright 2020 Simon Hoffmann
#pragma once
#include <ros/ros.h>
#include <vector>
#include <string>
#include <memory>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tod_helper/vehicle/Model.h>
#include "tod_msgs/Trajectory.h"
#include "ros/debug.h"
#include "tod_msgs/PrimaryControlCmd.h"
#include "tod_core/VehicleParameters.h"
#include "LateralPlanner.h"

class ClothoidPlanner : public LateralPlanner {
    public:
        explicit ClothoidPlanner(std::shared_ptr<tod_core::VehicleParameters> vehParams, float segmentLength);
        tod_msgs::Trajectory calculate_path(const tod_msgs::PrimaryControlCmd& cmd,
            const std::string& frame_id, const std::string& child_frame_id, const double length) override;
        void update_pose(const double x, const double y, const double yaw) override;

    private:
        std::vector<geometry_msgs::Point> numeric_integration_position(const double c_0, const double c_1,
                const double length, const int n_increments);
        constexpr double calc_orientation(const double c_0, const double c_1, const double s,
            const double yaw_0) noexcept;
        double calc_curvature_rate(const float curvature, const u_int64_t timeStampNs, const double velocity);
        constexpr double get_arc_length_from_pose(const int trajectoryElement, const int total_elements,
            const double length) noexcept;
        double _offsetOrientation;
        geometry_msgs::Point _offsetPosition;
};
