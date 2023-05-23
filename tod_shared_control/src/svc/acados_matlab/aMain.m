clear all; close all; clc;
% working on commit d65fb0ee114b2192aeb8b85788c84581540066bc
env_run = getenv('ENV_RUN'); % check that env.sh has been run
if (~strcmp(env_run, 'true'))
    error('env.sh has not been sourced! Before executing this example, run: source env.sh');
end

GENERATE_C_CODE = false;

% prediction horizon
ocp_N = 40; 
h = 0.050; T = ocp_N*h; % horizon length time

% solver
model = create_optimization_problem();
[ocp_opts,cost_type] = create_ocp_opts(ocp_N);
ocp_model = create_ocp_model(model, ocp_opts, T, cost_type); 
ocp = acados_ocp(ocp_model, ocp_opts);
if GENERATE_C_CODE
    ocp.generate_c_code()
end

% simulation
[sim_model,sim_opts] = create_sim_model(model, ocp_opts, ocp_N, T);
sim = acados_sim(sim_model, sim_opts);
run_simulation;
