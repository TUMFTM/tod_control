function [ocp_opts,cost_type] = create_ocp_opts(ocp_N)
% ocp solver options
nlp_solver = 'sqp'; % sqp / sqp_rti
nlp_solver_exact_hessian = 'false'; % true (cannot generate c code) / false

% sqp solver settings
myNLPtolerance = 1e-2;
nlp_solver_max_iter = 15;
nlp_solver_tol_stat = myNLPtolerance;
nlp_solver_tol_eq = myNLPtolerance;
nlp_solver_tol_ineq = myNLPtolerance;
nlp_solver_tol_comp = myNLPtolerance;

% qp solver settings,
myQPtolerance = 1e-4; % keep smaller 1e-3
qp_solver_iter_max = 100;
qp_solver_tol_stat = myQPtolerance;
qp_solver_tol_eq = myQPtolerance;
qp_solver_tol_ineq = myQPtolerance;
qp_solver_tol_comp = myQPtolerance;


%%%%%%%%%%%%5%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% for the most part, I have no idea what I am doing %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
qp_solver = 'partial_condensing_hpipm';
% full_condensing_hpipm, partial_condensing_hpipm, full_condensing_qpoases,
% partial_condensing_osqp
qp_solver_cond_N = 5;
qp_solver_warm_start = 0;
qp_solver_cond_ric_alg = 0; % 0: dont factorize hessian in the condensing; 1: factorize
qp_solver_ric_alg = 0; % HPIPM specific

param_scheme = 'multiple_shooting_unif_grid';
regularize_method = 'project_reduc_hess'; % no_regularize, project, ...
% project_reduc_hess, mirror, convexify
ocp_sim_method = 'erk'; % erk, irk, irk_gnsf
ocp_sim_method_num_stages = 4;
ocp_sim_method_num_steps = 1;
cost_type = 'linear_ls'; % linear_ls, ext_cost

% compile and codegen
compile_interface = 'auto'; % true, false
codgen_model = 'true'; % true, false



%%%%%%%%%%%%%%%%%%%%%%
% populate structure %
%%%%%%%%%%%%%%%%%%%%%%
ocp_opts = acados_ocp_opts();
ocp_opts.set('compile_interface', compile_interface); % true, false
ocp_opts.set('codgen_model', codgen_model); % true, false
ocp_opts.set('param_scheme', param_scheme);
ocp_opts.set('param_scheme_N', ocp_N);
ocp_opts.set('nlp_solver', nlp_solver);
ocp_opts.set('nlp_solver_exact_hessian', nlp_solver_exact_hessian);
ocp_opts.set('regularize_method', regularize_method);
if (strcmp(nlp_solver, 'sqp'))
    ocp_opts.set('nlp_solver_max_iter', nlp_solver_max_iter);
end
ocp_opts.set('nlp_solver_tol_stat', nlp_solver_tol_stat);
ocp_opts.set('nlp_solver_tol_eq', nlp_solver_tol_eq);
ocp_opts.set('nlp_solver_tol_ineq', nlp_solver_tol_ineq);
ocp_opts.set('nlp_solver_tol_comp', nlp_solver_tol_comp);
ocp_opts.set('qp_solver', qp_solver);
if (strcmp(qp_solver, 'partial_condensing_hpipm'))
    ocp_opts.set('qp_solver_cond_N', qp_solver_cond_N);
    ocp_opts.set('qp_solver_cond_ric_alg', qp_solver_cond_ric_alg);
    ocp_opts.set('qp_solver_ric_alg', qp_solver_ric_alg);
    ocp_opts.set('qp_solver_warm_start', qp_solver_warm_start);
    
    ocp_opts.set('qp_solver_tol_stat',  qp_solver_tol_stat);
    ocp_opts.set('qp_solver_tol_eq',  qp_solver_tol_eq);
    ocp_opts.set('qp_solver_tol_ineq',  qp_solver_tol_ineq);
    ocp_opts.set('qp_solver_tol_comp',  qp_solver_tol_comp);
end
ocp_opts.set('qp_solver_iter_max', qp_solver_iter_max);
ocp_opts.set('sim_method', ocp_sim_method);
ocp_opts.set('sim_method_num_stages', ocp_sim_method_num_stages);
ocp_opts.set('sim_method_num_steps', ocp_sim_method_num_steps);

% ocp_opts.opts_struct

end
