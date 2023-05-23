// Copyright 2021 Andreas Schimpe
#pragma once
#include "c_generated_code/acados_solver_ocp_vehicle.h"
#include "acados/SolverBase.h"
#include <vector>

namespace tod_shared_control {

class VelocityOptimizer : public SolverBase {
public:
    struct Constraints { double velMax, accMin, accMax, latAccMax, jerkMax; };
    struct Parameters { double safeProgress; std::vector<double> curvatures; };

    VelocityOptimizer(const Constraints &constraints);
    ~VelocityOptimizer() { ocp_vehicle_acados_free_capsule(_acados_ocp_capsule); }
    const Constraints& get_constraints() const { return _constraints; }

    bool solve(const double currentVelocity, const double targetVelocity,
               const double currentProgress, const Parameters &parameters,
               double &outJerk, double &outAcceleration, double &outVelocity);

    void get_optimized_profiles(std::vector<double> &progressProfile,
                                std::vector<double> &velocityProfile,
                                std::vector<double> &accelerationProfile,
                                std::vector<double> &jerkProfile) const;



private:
    Constraints _constraints;
    nlp_solver_capsule *_acados_ocp_capsule{nullptr};
    int _mpc_np{0};
    double _previousAcceleration{0.0};

    void set_costs();
    void set_constraints();
    void update_initial_state(const double currentVelocity, const double currentProgress);
    void update_references(const double targetVelocity);
    void update_parameters(const Parameters &parameters);
};

} // namespace tod_shared_control
