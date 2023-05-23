// Copyright 2020 Andreas Schimpe
#include "MpcSolver.h"

namespace tod_shared_control {

MpcSolver::MpcSolver(std::shared_ptr<Parameters> params) : _params{params} {
    acados_ocp_capsule = ocp_vehicle_acados_create_capsule();
    int status = ocp_vehicle_acados_create(acados_ocp_capsule);
    if (status) {
        ROS_ERROR("MpcSolver: ocp_vehicle_acados_create() returned status %d. Exiting.\n", status);
        return;
    }
    nlp_config = ocp_vehicle_acados_get_nlp_config(acados_ocp_capsule);
    nlp_dims = ocp_vehicle_acados_get_nlp_dims(acados_ocp_capsule);
    nlp_in = ocp_vehicle_acados_get_nlp_in(acados_ocp_capsule);
    nlp_out = ocp_vehicle_acados_get_nlp_out(acados_ocp_capsule);
    nlp_solver = ocp_vehicle_acados_get_nlp_solver(acados_ocp_capsule);
    nlp_opts = ocp_vehicle_acados_get_nlp_opts(acados_ocp_capsule);
    read_nlp_dims_and_init_trajectories();
    _nofObstacles = _mpc_ns / 4; // four circles per obstacle
    _mpc_np = _nofObstacles * 5; // (x,y,a,b,phi) per obstacle

    // constraints
    std::vector<double> lh(_mpc_nh, 0.0);
    lh.at(0) = -params->vehParams->get_max_rwa_rad();
    lh.at(1) = -params->steeringRateMax;
    lh.at(2) = params->accelerationMin;
    for (int i=3; i < _mpc_nh; ++i) {
        lh.at(i) = 0.0; // do not cross ellipse
    }

    std::vector<double> uh(_mpc_nh, 0.0);
    uh.at(0) = params->vehParams->get_max_rwa_rad();
    uh.at(1) = params->steeringRateMax;
    uh.at(2) = params->accelerationMax;
    for (int i=3; i < _mpc_nh; ++i) {
        uh.at(i) = 100000000.0; // inf
    }

    for (int i = 0; i < _mpc_Np; i++) {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lh", lh.data());
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "uh", uh.data());
    }

    // initial cost
    std::vector<double> W0(_mpc_ny*_mpc_ny, 0.0);
    W0.at(0+_mpc_ny * 0) = params->steeringRateCost;
    W0.at(1+_mpc_ny * 1) = params->accelerationCost;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "W", W0.data());

    for (int i=1; i < _mpc_Np; ++i) {
        std::vector<double> W(_mpc_ny*_mpc_ny, 0);
        W.at(0+_mpc_ny * 0) = params->steeringRateCost;
        W.at(1+_mpc_ny * 1) = params->accelerationCost;
        W.at(5+_mpc_ny * 5) = std::exp(-params->cmdCostDecay * (i-1)) * params->steeringTrackingCost;
        W.at(6+_mpc_ny * 6) = std::exp(-params->cmdCostDecay * (i-1)) * params->velocityTrackingCost;
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "W", W.data());
    }

    // terminal cost
    std::vector<double> W_e(_mpc_nx*_mpc_nx, 0.0);
    W_e.at(4+(_mpc_nx) * 4) = 1000;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, _mpc_Np, "W", W_e.data());

    _solverReady = true;
}

bool MpcSolver::solve(const BicycleState &stateRef, const BicycleState &stateInit, const ObjectList &objects,
                      tod_msgs::ColoredPolygon &outPredictedPolygon, BicycleCommand &cmdShared,
                      std::vector<double> &outXTraj, std::vector<double> &outUTraj,
                      AcadosDebug &outAcadosDebug) {
    ros::Time t0 = ros::Time::now();
    update_object_params(objects);
    update_references(stateRef);
    update_initial_state(stateInit);
    int status = ocp_vehicle_acados_solve(acados_ocp_capsule);
    read_state_trajectories();
    outXTraj = _xtraj;
    outUTraj = _utraj;
    get_command(outXTraj, outUTraj, cmdShared);
    set_predicted_polygon(outXTraj, stateRef.Cmd, outPredictedPolygon);
    outAcadosDebug = get_acados_debug((ros::Time::now() - t0).toSec() * 1000.0, status);
    return !(status == ACADOS_FAILURE || status == ACADOS_QP_FAILURE);
}

void MpcSolver::update_object_params(const ObjectList &objects) {
    if (objects.size() != _nofObstacles) {
        ROS_WARN("Solver::update() - received %d objects - expected and taking only %d",
                 objects.size(), _nofObstacles);
    }
    // set parameters
    std::vector<double> p(_mpc_np, 0.0);
    for (int i = 0; i < _nofObstacles; ++i) {
        const auto &object = objects.at(i);
        p.at(i*5+0) = object.distCenterX; // x pos of ellipse
        p.at(i*5+1) = object.distCenterY; // y pos of ellipse
        p.at(i*5+2) = object.dimX / 2.0; // a of ellipse
        p.at(i*5+3) = object.dimY / 2.0; // b of ellipse
        p.at(i*5+4) = object.yawAngle; // heading of ellipse
    }
    for (int ii = 0; ii <= _mpc_Np; ii++) {
        ocp_vehicle_acados_update_params(acados_ocp_capsule, ii, p.data(), _mpc_np);
    }
}

void MpcSolver::update_references(const BicycleState &stateRef) {
    // stage reference for inputs and states
    std::vector<double> yref(_mpc_nx+_mpc_nu, 0.0);
    yref.at(0) = 0.0; // steering rate
    yref.at(1) = 0.0; // acceleration
    yref.at(2) = stateRef.PosX;
    yref.at(3) = stateRef.PosY;
    yref.at(4) = stateRef.Heading;
    yref.at(5) = stateRef.Cmd.Steering;
    yref.at(6) = stateRef.Cmd.Velocity;
    for (int i = 0; i < _mpc_Np; ++i) {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref.data());
    }
    // terminal reference for states
    std::vector<double> yref_e(_mpc_nx, 0.0);
    yref_e.at(0) = stateRef.PosX; // check penalties for what reference is actually tracked
    yref_e.at(1) = stateRef.PosY;
    yref_e.at(2) = stateRef.Heading;
    yref_e.at(3) = stateRef.Cmd.Steering;
    yref_e.at(4) = 0.0; // zero->robustness - else stateRef.command.velocity
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, _mpc_Np, "yref", yref_e.data());
}

void MpcSolver::set_predicted_polygon(const std::vector<double> xtraj_pred, const BicycleCommand &ref,
                                      tod_msgs::ColoredPolygon &predictedPolygon) {
    std::vector<tod_msgs::ColoredPoint> predsRight;
    double accSteeringIntervention{0.0};
    for (size_t i=0; i <= _mpc_Np; ++i) {
        double xpos = xtraj_pred.at(i * _mpc_nx);
        double ypos = xtraj_pred.at(i * _mpc_nx + 1);
        double theta = xtraj_pred.at(i * _mpc_nx + 2);
        double xfl, yfl, xfr, yfr;
        tod_helper::Vehicle::Model::calc_vehicle_front_edges(
            xpos, ypos, theta, _params->vehParams->get_distance_front_axle(),
            _params->vehParams->get_width(), xfl, yfl, xfr, yfr);
        tod_msgs::ColoredPoint& lcpt = predictedPolygon.points.emplace_back();
        tod_msgs::ColoredPoint& rcpt = predsRight.emplace_back();
        lcpt.point.x = (float) xfl;
        lcpt.point.y = (float) yfl;
        rcpt.point.x = (float) xfr;
        rcpt.point.y = (float) yfr;

        // intervention dependent coloring
        accSteeringIntervention += std::abs(ref.Steering - xtraj_pred.at(i * _mpc_nx + 3));
        accSteeringIntervention = std::min(accSteeringIntervention, 1.0);
        lcpt.color.r = rcpt.color.r = (float) accSteeringIntervention;
        lcpt.color.g = rcpt.color.g = 1.0f - (float) accSteeringIntervention;
        lcpt.color.a = rcpt.color.a = 1.0f;
    }
    std::reverse(predsRight.begin(), predsRight.end());
    predictedPolygon.points.insert(predictedPolygon.points.end(), predsRight.begin(), predsRight.end());
}

}; // namespace tod_shared_control
