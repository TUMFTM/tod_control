// Copyright 2021 Andreas Schimpe
#pragma once
#include <ros/ros.h>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <tod_msgs/PrimaryControlCmd.h>
#include <tod_msgs/Status.h>
#include <tod_msgs/VehicleData.h>
#include <nav_msgs/Odometry.h>
#include <tod_helper/geometry/Helpers.h>
#include "TrajectoryHandler.h"
#include "PtcSolver.h"
#include <tod_core/VehicleParameters.h>
#include <visualization_msgs/MarkerArray.h>
#include <tod_helper/trajectory/Helpers.h>
#include <std_msgs/Float32.h>
#include "ControllerBase.h"

namespace tod_shared_control {

class PathTrackingController : public ControllerBase {
public:
    explicit PathTrackingController(ros::NodeHandle& nodeHandle);
    ~PathTrackingController() override { }
    void run();

private:
    ros::Publisher _pubReference, _pubPredictions, _pubSolverTime, _pubTrackingError;
    std::shared_ptr<TrajectoryHandler> _trajHandler{nullptr};
    // params
    std::unique_ptr<tod_core::VehicleParameters> _vehicleParams;
    std::shared_ptr<PtcSolver::Parameters> _ctrlerParams{nullptr};
    // solver
    std::unique_ptr<PtcSolver> _solver{nullptr};

    // fcns
    tod_msgs::PrimaryControlCmd execute_control();
    BicycleState get_initial_state();
    std::vector<BicycleState> get_reference_states(const BicycleState &stateInit, double &outTrackingError);
    void publish_reference_and_predictions(const std::vector<BicycleState> &reference,
                                           const std::vector<BicycleState> &predictions);
    void publish_solver_time(const SolverBase::AcadosDebug &acadosDebug);
    void publish_tracking_error(const double trackingError);
    tod_msgs::Trajectory states_to_trajectory(const std::vector<BicycleState> &states);

    bool reached_final_waypoint(const BicycleState &init, const BicycleState &final) { return (init.distance_to(final) < 0.01); }
};

} // namespace tod_shared_control
