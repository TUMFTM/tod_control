function mdl = create_optimization_problem()
import casadi.*
mdl.name = 'ocp_vehicle';
mdl.x0 = [0; 0.01; 0.0; 0; 3]; % initial state

% SYSTEM PARAMETERS AND VARIABLES
lf = 1.446;
lr = 1.556;
lfd = 2.408;
lrd = 2.681;
wd = 2.177;

% named symbolic variables
x = SX.sym('x');
y = SX.sym('y');
theta = SX.sym('theta');
delta = SX.sym('delta');
v = SX.sym('v');
ddelta = SX.sym('ddelta');
a = SX.sym('a');

% (unnamed) symbolic variables and problem dims
sym_x = vertcat(x,y,theta,delta,v);
sym_u = vertcat(ddelta,a);
nx = length(sym_x);
nu = length(sym_u);
ny = nx + nu; ny_e = nx;
sym_xdot = SX.sym('xdot', nx, 1);


% SYSTEM DYNAMICS
beta = atan(lr * tan(delta) / (lf+lr));
expr_f_expl = vertcat(...
    v * cos(theta + beta), ...
    v * sin(theta + beta), ...
    v * sin(beta) / lr, ...
    ddelta, ...
    a);
expr_f_impl = expr_f_expl - sym_xdot;


% CONSTRAINTS
% obstacles
xo_ell = [63, 101.0, 137, 80, 170];
yo_ell = [6.5, 16.35, 27.5, 12.1, 17.6];
a_ell = [3, 2.5, 35, 0.7, 1.9];
b_ell = [1.5, 3.0, 5, 0.7, 1.9];
phi_ell = deg2rad([12.5, 30.0, 7.5, 10.0, 15.0]);
n_ell = 2;

% paramaters
np = 5 * length(xo_ell);
p = SX.sym('p', np);
expr_h = vertcat(delta, ddelta, a);
nhh = length(expr_h); 

rotEgo = [cos(theta) , -sin(theta) ; sin(theta) , cos(theta)];
rearBumper = [x;y] + rotEgo * [-lrd ; 0];
frontBumper = [x;y] + rotEgo * [lfd ; 0];
endShift = 0.15;
interShift = (1 - 2*endShift)/3;
shifts = [endShift , endShift + interShift , endShift + 2*interShift, 1-endShift];
nofC = length(shifts);
disks = [ ...
    rearBumper + shifts(1) * (frontBumper-rearBumper) , ...
    rearBumper + shifts(2) * (frontBumper-rearBumper) , ...
    rearBumper + shifts(3) * (frontBumper-rearBumper) , ...
    rearBumper + shifts(4) * (frontBumper-rearBumper) ];
radius = 1.2 * wd / 2; 

for i = 1 : length(xo_ell)
    xell = p((i-1)*5+1);
    yell = p((i-1)*5+2);
    aell = p((i-1)*5+3);
    bell = p((i-1)*5+4);
    phiell = p((i-1)*5+5);
    rotell = [cos(phiell) , -sin(phiell) ; sin(phiell) , cos(phiell)]; 
    for j = 1 : length(disks)
        dx = disks(1,j) - xell;
        dy = disks(2,j) - yell;
        ell = ((dx*cos(phiell)-dy*sin(phiell)) / (aell+radius))^n_ell + ...
            ((dx*sin(phiell)+dy*cos(phiell))/(bell+radius))^n_ell - 1; 
        expr_h = vertcat(expr_h, ell); 
    end
end
nh = length(expr_h);
nsh = nh - nhh;


% COST FUNCTION
% linear least square cost: y^T * W * y, where y = Vx * x + Vu * u - y_ref
% input-to-output matrix in lagrange term
Vu = zeros(ny, nu); for ii=1:nu Vu(ii,ii) = 1.0; end
% state-to-output matrix in lagrange term
Vx = zeros(ny, nx); for ii=1:nx Vx(nu+ii,ii) = 1.0; end
% state-to-output matrix in mayer term
Vx_e = zeros(ny_e, nx); for ii=1:nx Vx_e(ii,ii) = 1.0; end

% weight matrices
W = eye(ny); % weight matrix in each stage
W(1,1) = 0.01; % penalty: ddelta
W(2,2) = 0.01; % penalty: a
W(3,3) = 0.0; % penalty: x
W(4,4) = 0.0; % penalty: y
W(5,5) = 0.0; % penalty: theta
W(6,6) = 200; % penalty: delta
W(7,7) = 2; % penality: v
% weight matrix in terminal stage
W_e = W(nu+1:nu+nx, nu+1:nu+nx);

% references
yr = [0;0;0;0;0;0;3]; % stage reference
yr_e = yr(nu+1:ny); % terminal stage reference

% slack penalties (recommendation: Z^2 < z)
Z = 10 * eye(nsh);
z = 1000 * ones(nsh,1);


% CONSTRAINTS
delta_max = 2*deg2rad(20);
ddelta_max = 2*deg2rad(30);
a_max = 3.5;
ubu = [delta_max; ddelta_max; a_max];
ubu = [ubu ; 1e8*ones(length(expr_h) - nhh,1)];
lbu = -ubu; lbu(nhh + 1:end) = 0;
mdl.ubu = ubu; mdl.lbu = lbu;

% soft constraints on delta deviation and obstacles
mdl.Jsh = zeros(nh, nsh); % idxs 1, 2, 3 are not soft constrs on de, dde, a
for i = 1 : nsh mdl.Jsh(nhh+i, i) = 1.0; end


% POPULATE STRUCTURE
mdl.nx = nx;
mdl.nu = nu;
mdl.np = np;
mdl.ny = nu + nx;
mdl.ny_e = ny_e;
mdl.sym_x = sym_x;
mdl.sym_xdot = sym_xdot;
mdl.sym_u = sym_u;
mdl.sym_p = p;
mdl.expr_f_expl = expr_f_expl;
mdl.expr_f_impl = expr_f_impl;
mdl.expr_h = expr_h;
mdl.xo_ell = xo_ell;
mdl.yo_ell = yo_ell;
mdl.a_ell = a_ell;
mdl.b_ell = b_ell;
mdl.phi_ell = phi_ell; 
mdl.n_ell = n_ell;
mdl.radius = radius; 
mdl.W = W;
mdl.W_e = W_e;
mdl.Z = Z;
mdl.yr = yr;
mdl.yr_e = yr_e;
mdl.z = z;
mdl.ng = 0;
mdl.ng_e = 0;
mdl.nbx = 0;
mdl.nbu = 0;
mdl.nh = nh;
mdl.nh_e = 0;
mdl.nsh = nsh;
mdl.Vu = Vu;
mdl.Vx = Vx;
mdl.Vx_e = Vx_e;

end
