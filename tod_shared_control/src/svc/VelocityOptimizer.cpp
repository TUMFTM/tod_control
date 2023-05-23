// Copyright 2021 Andreas Schimpe
#include "VelocityOptimizer.h"

namespace tod_shared_control {

VelocityOptimizer::VelocityOptimizer(const Constraints &constraints) :
    _constraints{constraints},
    _acados_ocp_capsule{ocp_vehicle_acados_create_capsule()} {
    int status = ocp_vehicle_acados_create(_acados_ocp_capsule);
    if (status) {
        printf("ocp_vehicle_acados_create() returned status %d. Exiting.\n", status);
        return;
    }
    nlp_config = ocp_vehicle_acados_get_nlp_config(_acados_ocp_capsule);
    nlp_dims = ocp_vehicle_acados_get_nlp_dims(_acados_ocp_capsule);
    nlp_in = ocp_vehicle_acados_get_nlp_in(_acados_ocp_capsule);
    nlp_out = ocp_vehicle_acados_get_nlp_out(_acados_ocp_capsule);
    nlp_solver = ocp_vehicle_acados_get_nlp_solver(_acados_ocp_capsule);
    nlp_opts = ocp_vehicle_acados_get_nlp_opts(_acados_ocp_capsule);
    read_nlp_dims_and_init_trajectories();
    _mpc_np = 2;

    set_costs();
    set_constraints();
    _solverReady = true;
}

bool VelocityOptimizer::solve(const double currentVelocity, const double targetVelocity,
                              const double currentProgress, const Parameters &parameters,
                              double &outJerk, double &outAcceleration, double &outVelocity) {
    update_initial_state(currentVelocity, currentProgress);
    update_references(targetVelocity);
    update_parameters(parameters);

    int status = ocp_vehicle_acados_solve(_acados_ocp_capsule);
    read_state_trajectories();
    outVelocity = _xtraj.at(_mpc_nx + 1);
    outAcceleration = _xtraj.at(_mpc_nx + 2);
    outJerk = _utraj.at(0);
    _previousAcceleration = outAcceleration;
    return !(status == ACADOS_FAILURE || status == ACADOS_QP_FAILURE);
}

void VelocityOptimizer::get_optimized_profiles(std::vector<double> &progressProfile,
                                               std::vector<double> &velocityProfile,
                                               std::vector<double> &accelerationProfile,
                                               std::vector<double> &jerkProfile) const {
    progressProfile.clear();
    velocityProfile.clear();
    accelerationProfile.clear();
    jerkProfile.clear();;
    for (int stage = 0; stage <= _mpc_Np; ++stage) {
        progressProfile.push_back(_xtraj.at(stage*_mpc_nx + 0));
        velocityProfile.push_back(_xtraj.at(stage*_mpc_nx + 1));
        accelerationProfile.push_back(_xtraj.at(stage*_mpc_nx + 2));
        if (stage < _mpc_Np) {
            jerkProfile.push_back(_utraj.at(stage*_mpc_nu + 0));
        }
    }
}

void VelocityOptimizer::set_costs() {
    // stage cost for use of jerk and acceleration
    std::vector<double> W(_mpc_ny * _mpc_ny, 0.0);
    for (int stage=0; stage < _mpc_Np; ++stage) {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, stage, "W", W.data());
    }
    // stage 1 cost for deviating from target velocity
    W.at(2 + _mpc_ny * 2) = 500.0;
    int stage = 1;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, stage, "W", W.data());
    // terminal cost for deviating from terminal velocity (zero)
    std::vector<double> W_e(_mpc_nx * _mpc_nx, 0.0);
    W_e.at(1 + _mpc_nx * 1) = 10000.0;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, _mpc_Np, "W", W_e.data());
}

void VelocityOptimizer::set_constraints() {
    double velMin{-0.1};
    double minProgress{-100.0}, maxProgress{0.0}; // progress minus param safe progress
    std::vector<double> lh = {velMin, _constraints.accMin, -_constraints.jerkMax, -_constraints.latAccMax, minProgress}; // v,a,j,a_lat,s-s_safe
    std::vector<double> uh = {_constraints.velMax, _constraints.accMax, +_constraints.jerkMax, +_constraints.latAccMax, maxProgress};
    for (int stage = 0; stage <= _mpc_Np; ++stage) {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, stage, "lh", lh.data());
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, stage, "uh", uh.data());
    }
}

void VelocityOptimizer::update_initial_state(const double currentVelocity, const double currentProgress) {
    std::vector<double> x0 = {currentProgress, currentVelocity, _previousAcceleration}; // s,v,a
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", x0.data());
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", x0.data());
}

void VelocityOptimizer::update_references(const double targetVelocity) {
    std::vector<double> yref = {0.0, 0.0, targetVelocity, 0.0}; // j,s,v,a
    for (int stage = 0; stage < _mpc_Np; ++stage) {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, stage, "yref", yref.data());
    }
    std::vector<double> yref_e = {0.0, 0.0, 0.0}; // s,v,a
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, _mpc_Np, "yref", yref_e.data());
}

void VelocityOptimizer::update_parameters(const Parameters &parameters) {
    for (int stage = 0; stage <= _mpc_Np; ++stage) {
        std::vector<double> p = {parameters.curvatures.at(stage), parameters.safeProgress};
        ocp_vehicle_acados_update_params(_acados_ocp_capsule, stage, p.data(), _mpc_np);
    }
}

} // namespace tod_shared_control
