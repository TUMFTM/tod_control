// Copyright 2020 Andreas Schimpe
#pragma once
#include <memory>
#include "c_generated_code/acados_solver_ocp_vehicle.h"
#include "acados/BicycleSolver.h"

namespace tod_shared_control {

class PtcSolver : public BicycleSolver {
public:
    struct Parameters {
        // constraints
        double steeringMax{0.0};
        double steeringRateMax{0.0};
        double accelerationMin{0.0}, accelerationMax{0.0};

        // penalties
        double steeringRateCost{10.0};
        double accelerationCost{1.0};
        double xTrackingCost{2.0};
        double yTrackingCost{2.0};
        double headingTrackingCost{0.01};
        double steeringTrackingCost{0.01};
        double velocityTrackingCost{0.01};
    };

    explicit PtcSolver(std::shared_ptr<Parameters> params);
    ~PtcSolver() { ocp_vehicle_acados_free_capsule(acados_ocp_capsule); }

    bool solve(const std::vector<BicycleState> &stateReference, const BicycleState &stateInit,
               BicycleCommand &command, std::vector<BicycleState> &outPredictions, AcadosDebug &outAcadosDebug);

private:
    std::shared_ptr<Parameters> _params{nullptr};
    nlp_solver_capsule *acados_ocp_capsule{nullptr};
};

} // namespace tod_shared_control
