% test of native matlab interface
clear all; close all; clc;
% working on commit d65fb0ee114b2192aeb8b85788c84581540066bc
env_run = getenv('ENV_RUN'); % check that env.sh has been run
if (~strcmp(env_run, 'true'))
    error('env.sh has not been sourced! Before executing this example, run: source env.sh');
end

GENERATE_C_CODE = false;

% Optimization Problem
bProblem_kinematic_vehicle_model;
bProblem_cost_function;
bProblem_constraints;

% prediction horizon
ocp_N = 40; 
h = 0.05; T = ocp_N*h; % horizon length time

% Solver
cSolver_settings; 
cSolver_create;
ocp = acados_ocp(ocp_model, ocp_opts);
if GENERATE_C_CODE
    ocp.generate_c_code()
end
