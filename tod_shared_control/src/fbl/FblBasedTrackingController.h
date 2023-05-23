// Copyright 2021 Andreas Schimpe
#pragma once
#include <ros/ros.h>
#include "ControllerBase.h"
#include "TrajectoryHandler.h"
#include "FeedbackLinearization.h"
#include <tod_core/VehicleParameters.h>
#include <tod_shared_control/FblLog.h>
#include <memory>
#include <string>

namespace tod_shared_control {

class FblBasedTrackingController : public ControllerBase {
public:
    explicit FblBasedTrackingController(ros::NodeHandle& nodeHandle);
    ~FblBasedTrackingController() override { }
    void run() override;

private:
    ros::Publisher _pubLog;
    std::unique_ptr<tod_core::VehicleParameters> _vehicleParams;
    std::unique_ptr<TrajectoryHandler> _trajHandler{nullptr};
    FeedbackLinearization _fbl;

    tod_msgs::PrimaryControlCmd  execute_control(const double dt);
    void publish_log(FblLog &log);
};

} // namespace tod_shared_control
