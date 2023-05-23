// Copyright 2020 Andreas Schimpe
#include "PtcSolver.h"

namespace tod_shared_control {

PtcSolver::PtcSolver(std::shared_ptr<Parameters> params) : _params{params} {
    acados_ocp_capsule = ocp_vehicle_acados_create_capsule();
    int status = ocp_vehicle_acados_create(acados_ocp_capsule);
    if (status) {
        printf("PtcSolver: ocp_vehicle_acados_create() returned status %d. Exiting.\n", status);
        return;
    }
    nlp_config = ocp_vehicle_acados_get_nlp_config(acados_ocp_capsule);
    nlp_dims = ocp_vehicle_acados_get_nlp_dims(acados_ocp_capsule);
    nlp_in = ocp_vehicle_acados_get_nlp_in(acados_ocp_capsule);
    nlp_out = ocp_vehicle_acados_get_nlp_out(acados_ocp_capsule);
    nlp_solver = ocp_vehicle_acados_get_nlp_solver(acados_ocp_capsule);
    nlp_opts = ocp_vehicle_acados_get_nlp_opts(acados_ocp_capsule);
    read_nlp_dims_and_init_trajectories();


    // constraints
    double lh[_mpc_nh];
    lh[0] = -params->steeringMax;
    lh[1] = -params->steeringRateMax;
    lh[2] = params->accelerationMin;
    double uh[_mpc_nh];
    uh[0] = params->steeringMax;
    uh[1] = params->steeringRateMax;
    uh[2] = params->accelerationMax;
    for (int i = 0; i < _mpc_Np; i++) {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lh", lh);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "uh", uh);
    }


    // cost function
    std::vector<double> W(_mpc_ny*_mpc_ny, 0.0);
    W.at(0+_mpc_ny * 0) = params->steeringRateCost;
    W.at(1+_mpc_ny * 1) = params->accelerationCost;
    W.at(2+_mpc_ny * 2) = params->xTrackingCost;
    W.at(3+_mpc_ny * 3) = params->yTrackingCost;
    W.at(4+_mpc_ny * 4) = params->headingTrackingCost;
    W.at(5+_mpc_ny * 5) = params->steeringTrackingCost;
    W.at(6+_mpc_ny * 6) = params->velocityTrackingCost;
    for (int i=0; i < _mpc_Np; ++i)
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "W", W.data());

    // terminal cost
    std::vector<double> W_e(_mpc_nx*_mpc_nx, 0.0);
    W_e.at(0+_mpc_nx * 0) = params->xTrackingCost;
    W_e.at(1+_mpc_nx * 1) = params->yTrackingCost;
    W_e.at(2+_mpc_nx * 2) = params->headingTrackingCost;
    W_e.at(3+_mpc_nx * 3) = params->steeringTrackingCost;
    W_e.at(4+_mpc_nx * 4) = params->velocityTrackingCost;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, _mpc_Np, "W", W_e.data());

    _solverReady = true;
}

bool PtcSolver::solve(const std::vector<BicycleState> &stateReference, const BicycleState &stateInit,
                      BicycleCommand &command, std::vector<BicycleState> &outPredictions, AcadosDebug &outAcadosDebug) {
    update_references(stateReference);
    update_initial_state(stateInit);
    int status = ocp_vehicle_acados_solve(acados_ocp_capsule);
    read_state_trajectories();
    get_command(_xtraj, _utraj, command);
    get_predictions(_xtraj, _utraj, outPredictions);
    return !(status == ACADOS_FAILURE || status == ACADOS_QP_FAILURE);
}

} // namespace tod_shared_control
