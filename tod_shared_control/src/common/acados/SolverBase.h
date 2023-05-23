// Copyright 2021 Andreas Schimpe
#pragma once
#include "acados/ocp_nlp/ocp_nlp_common.h"
#include "acados_c/ocp_nlp_interface.h"
#include <vector>

namespace tod_shared_control {

class SolverBase {
public:
    struct AcadosDebug {
        int nx{0}, nu{0}, N{0};
        int sqp_iterations{0};
        int solver_time_ms{0};
        int status{0};
    };

    bool ready() const { return _solverReady; }
    int get_prediction_horizon() const { return _mpc_Np; }
    double get_sampling_time() const { return _mpc_Ts; }
    bool cycle_time_consistent(const double cycleTimeSec) { return std::abs(cycleTimeSec - _mpc_Ts) < 0.01; }

    bool _solverReady{false};

    ocp_nlp_solver *nlp_solver{nullptr};
    ocp_nlp_config *nlp_config{nullptr};
    ocp_nlp_dims *nlp_dims{nullptr};
    ocp_nlp_in *nlp_in{nullptr};
    ocp_nlp_out *nlp_out{nullptr};
    void *nlp_opts{nullptr};

    // mpc problem dimensions and trajectories
    double _mpc_Ts{0.0};
    int _mpc_Np{0}, _mpc_nx{0}, _mpc_nu{0}, _mpc_nh{0}, _mpc_ns{0}, _mpc_ny{0};
    std::vector<double> _xtraj, _utraj;

    void read_nlp_dims_and_init_trajectories() {
        _mpc_Ts = *nlp_in->Ts;
        _mpc_Np = nlp_dims->N;
        _mpc_nx = *nlp_dims->nx;
        _mpc_nu = *nlp_dims->nu;
        _mpc_nh = *nlp_dims->ni - *nlp_dims->ns - *nlp_dims->nx;
        _mpc_ns = *nlp_dims->ns;
        _mpc_ny = _mpc_nx + _mpc_nu;
        _xtraj = std::vector(_mpc_nx * (_mpc_Np+1), 0.0);
        _utraj = std::vector(_mpc_nu * (_mpc_Np+1), 0.0);
    }

    AcadosDebug get_acados_debug(const double elapsedMS, const int status) const {
        AcadosDebug acadosDebug;
        acadosDebug.N = nlp_dims->N;
        acadosDebug.nu = *nlp_dims->nu;
        acadosDebug.nx = *nlp_dims->nx;
        ocp_nlp_get(nlp_config, nlp_solver, "sqp_iter", &acadosDebug.sqp_iterations);
        acadosDebug.solver_time_ms = elapsedMS;
        acadosDebug.status = status;
        return acadosDebug;
    }

    void read_state_trajectories() {
        for (int stage = 0; stage <= _mpc_Np; ++stage) {
            ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, stage, "x", &_xtraj.at(stage * _mpc_nx));
            ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, stage, "u", &_utraj.at(stage * _mpc_nu));
        }
    }
};

} // namespace tod_shared_control
