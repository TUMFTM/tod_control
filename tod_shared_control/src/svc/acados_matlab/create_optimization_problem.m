function model = create_optimization_problem()

model.name = 'ocp_vehicle';
import casadi.*

% named symbolic variables
s = SX.sym('s');
v = SX.sym('v');
a = SX.sym('a');
j = SX.sym('j'); 
kappa = SX.sym('kappa');
s_safe = SX.sym('s_safe'); 

% (unnamed) symbolic variables
sym_x = vertcat(s,v,a);
sym_u = vertcat(j);
sym_p = [kappa, s_safe]; 
model.x0 = [0.0; 3.0; 0.0]; % s,v,a

% system dimensions
nx = length(sym_x);
nu = length(sym_u);
np = length(sym_p) ;
sym_xdot = SX.sym('xdot', nx, 1);
if nx ~= length(model.x0)
    disp('mismatch in nx and length(x0)'); 
    return;
end

% dynamics
expr_f_expl = vertcat(v, a, j);
expr_f_impl = expr_f_expl - sym_xdot;

% constraints
expr_h = vertcat(v, a, j, kappa*v^2, s-s_safe);
model.ubu = [100.0 ; +2.0 ; +15.0 ; +5.0 ; 0.0];
model.lbu = [0.0   ; -3.5 ; -15.0 ; -5.0 ; -100.0]; 

% populate structure
model.nx = nx;
model.nu = nu;
model.np = np;
model.sym_x = sym_x;
model.sym_xdot = sym_xdot;
model.sym_u = sym_u;
model.sym_p = sym_p; 
model.expr_f_expl = expr_f_expl;
model.expr_f_impl = expr_f_impl;
model.expr_h = expr_h;

% dims
model.ny = nu + nx; % number of outputs in lagrange term
model.ny_e = nx; % number of outputs in mayer term
model.ng = 0; % number of general linear constraints intermediate stages
model.ng_e = 0; % number of general linear constraints final stage
model.nbx = 0; % number of bounds on state x
model.nbu = 0;
model.nh = length(model.expr_h);
model.nh_e = 0;
model.nsh = 2;

% soft constraints on jerk and acceleration (idxs 2 and 3 in expr_h
model.Jsh = zeros(model.nh, model.nsh);
for i = 1 : model.nsh model.Jsh(1+i, i) = 1.0; end
% slack penalties - recommendation: keep be Z^2 < z
model.Z = 10 * eye(model.nsh);
model.z = 1000 * ones(model.nsh,1);

% cost fcn
% references
model.yr = [ 0.0 ; 0.0 ; 5.0 ; 0.0 ]; % j,s,v,a
model.yr_e = [ 0.0 ; 0.0 ; 0.0 ]; % s,v,a
% weights
model.W = zeros(model.ny,model.ny); 
model.W(1,1) = 0.1; % j
model.W(3,3) = 10.0; % v
model.W(4,4) = 0.1; % a
model.W_e = zeros(model.nx,model.nx);
model.W_e(2,2) = 1000.0; % v_terminal should be zero 

% linear least square cost: y^T * W * y, where y = Vx * x + Vu * u - y_ref
% input-to-output matrix in lagrange term
model.Vu = zeros(model.ny, model.nu);
for ii=1:nu model.Vu(ii,ii) = 1.0; end
% state-to-output matrix in lagrange term
model.Vx = zeros(model.ny, model.nx);
for ii=1:model.nx model.Vx(model.nu+ii,ii) = 1.0; end
% state-to-output matrix in mayer term
model.Vx_e = zeros(model.ny_e, model.nx);
for ii=1:model.nx model.Vx_e(ii,ii) = 1.0; end

end
