// Copyright 2020 Andreas Schimpe
#pragma once
#include <ros/ros.h>
#include <tod_msgs/ColoredPolygon.h>
#include "ObjectsHandler.h"
#include "c_generated_code/acados_solver_ocp_vehicle.h"
#include "tod_helper/vehicle/Model.h"
#include "tod_core/VehicleParameters.h"
#include <memory>
#include <vector>
#include <algorithm>
#include <string>
#include "acados/BicycleSolver.h"

namespace tod_shared_control {

typedef std::vector<tod_msgs::ObjectData> ObjectList;

class MpcSolver : public BicycleSolver {
public:
    struct Parameters {
        // constraints
        double steeringRateMax{0.0};
        double accelerationMin{0.0};
        double accelerationMax{0.0};

        // penalties
        double steeringRateCost{0.0};
        double accelerationCost{0.0};
        double steeringTrackingCost{0.0};
        double velocityTrackingCost{0.0};
        double cmdCostDecay{0.0};

        std::unique_ptr<tod_core::VehicleParameters> vehParams;

        explicit Parameters(ros::NodeHandle &nh) :
            vehParams{std::make_unique<tod_core::VehicleParameters>(nh)} { }
    };

    explicit MpcSolver(std::shared_ptr<Parameters> params);
    ~MpcSolver() { ocp_vehicle_acados_free_capsule(acados_ocp_capsule); }

    int get_nof_obstacles() const { return _nofObstacles; }

    bool solve(const BicycleState &stateRef, const BicycleState &stateInit, const ObjectList &objects,
               tod_msgs::ColoredPolygon &outPredictedPolygon, BicycleCommand &cmdShared,
               std::vector<double> &outXTraj, std::vector<double> &outUTraj,
               AcadosDebug &outAcadosDebug);

private:
    std::shared_ptr<Parameters> _params{nullptr};
    int _nofObstacles{0}, _mpc_np{0};
    nlp_solver_capsule *acados_ocp_capsule{nullptr};

    void update_object_params(const ObjectList &objects);
    void update_references(const BicycleState &stateRef);
    void set_predicted_polygon(const std::vector<double> xtraj_pred, const BicycleCommand &ref,
                               tod_msgs::ColoredPolygon &predictedPolygon);
};

}; // namespace tod_shared_control
