// Copyright 2021 Andreas Schimpe
#pragma once
#include "SolverBase.h"
#include "../BicycleModelVariables.h"

namespace tod_shared_control {

class BicycleSolver : public SolverBase {
public:
    void get_command(const std::vector<double> &xtraj, const std::vector<double> &utraj, BicycleCommand &cmd) const {
        cmd.Steering = xtraj.at(_mpc_nx + 3);
        cmd.Velocity = xtraj.at(_mpc_nx + 4);
        cmd.input.SteeringRate = utraj.at(0);
        cmd.input.Acceleration = utraj.at(1);
    }

    void get_predictions(const std::vector<double> &xtraj, const std::vector<double> &utraj,
                                 std::vector<BicycleState> &statePredictions) const {
        statePredictions.clear();
        for (int i=0; i <= _mpc_Np; ++i) {
            BicycleState& st = statePredictions.emplace_back();
            st.PosX = xtraj.at(i * _mpc_nx + 0);
            st.PosY = xtraj.at(i * _mpc_nx + 1);
            st.Heading = xtraj.at(i * _mpc_nx + 2);
            st.Cmd.Steering = xtraj.at(i * _mpc_nx + 3);
            st.Cmd.Velocity = xtraj.at(i * _mpc_nx + 4);
            if (i < _mpc_Np) {
                st.Cmd.input.SteeringRate = utraj.at(i * _mpc_nu + 0);
                st.Cmd.input.Acceleration = utraj.at(i * _mpc_nu + 1);
            }
        }
    }

    void update_references(const std::vector<BicycleState> &stateReference) {
        // stage references
        for (int i = 0; i < _mpc_Np; ++i) {
            std::vector<double> yref(_mpc_nx + _mpc_nu);
            yref.at(0) = stateReference.at(i).Cmd.input.SteeringRate;
            yref.at(1) = stateReference.at(i).Cmd.input.Acceleration;
            yref.at(2) = stateReference.at(i).PosX;
            yref.at(3) = stateReference.at(i).PosY;
            yref.at(4) = stateReference.at(i).Heading;
            yref.at(5) = stateReference.at(i).Cmd.Steering;
            yref.at(6) = stateReference.at(i).Cmd.Velocity;
            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref.data());
        }
        // terminal reference
        std::vector<double> yref_e(_mpc_nx);
        yref_e.at(0) = stateReference.at(_mpc_Np).PosX;
        yref_e.at(1) = stateReference.at(_mpc_Np).PosY;
        yref_e.at(2) = stateReference.at(_mpc_Np).Heading;
        yref_e.at(3) = stateReference.at(_mpc_Np).Cmd.Steering;
        yref_e.at(4) = stateReference.at(_mpc_Np).Cmd.Velocity;
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, _mpc_Np, "yref", yref_e.data());
    }

    void update_initial_state(const BicycleState &stateInit) {
        std::vector<double> x0(_mpc_nx);
        x0.at(0) = stateInit.PosX;
        x0.at(1) = stateInit.PosY;
        x0.at(2) = stateInit.Heading;
        x0.at(3) = stateInit.Cmd.Steering;
        x0.at(4) = stateInit.Cmd.Velocity;
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", x0.data());
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", x0.data());
    }
};

} // namespace tod_shared_control
