model_name = 'ocp_vehicle';
x0 = [0; 0.01; 0.0; 0; 3]; % initial states

import casadi.*

% system dimensions
nx = 5;
nu = 2;

% system parameters
lf = 1.446;
lr = 1.556;

% named symbolic variables
x = SX.sym('x');
y = SX.sym('y');
theta = SX.sym('theta');
delta = SX.sym('delta');
v = SX.sym('v');
ddelta = SX.sym('ddelta');
a = SX.sym('a');

% (unnamed) symbolic variables
sym_x = vertcat(x,y,theta,delta,v);
sym_xdot = SX.sym('xdot', nx, 1);
sym_u = vertcat(ddelta,a);

% dynamics
beta = atan(lr * tan(delta) / (lf+lr));
expr_f_expl = vertcat(...
    v * cos(theta + beta), ...
    v * sin(theta + beta), ...
    v * sin(beta) / lr, ...
    ddelta, ...
    a);
expr_f_impl = expr_f_expl - sym_xdot;

expr_h = vertcat(delta, ddelta, a);

% populate structure
model.nx = nx;
model.nu = nu;
% model.np = np;
model.sym_x = sym_x;
model.sym_xdot = sym_xdot;
model.sym_u = sym_u;
% model.sym_p = p;
model.expr_f_expl = expr_f_expl;
model.expr_f_impl = expr_f_impl;
model.expr_h = expr_h;

% dims
ny = nu + nx; % number of outputs in lagrange term
ny_e = nx; % number of outputs in mayer term
ng = 0; % number of general linear constraints intermediate stages
ng_e = 0; % number of general linear constraints final stage
nbx = 0; % number of bounds on state x
nbu = 0;
nh = length(model.expr_h);
nh_e = 0;
