% cost
% linear least square cost: y^T * W * y, where y = Vx * x + Vu * u - y_ref
% input-to-output matrix in lagrange term
Vu = zeros(ny, nu);
for ii=1:nu Vu(ii,ii) = 1.0; end
% state-to-output matrix in lagrange term
Vx = zeros(ny, nx);
for ii=1:nx Vx(nu+ii,ii) = 1.0; end
% state-to-output matrix in mayer term
Vx_e = zeros(ny_e, nx);
for ii=1:nx Vx_e(ii,ii) = 1.0; end

W = eye(ny); % weight matrix in lagrange term
W(1,1) = 0.001; % penalty: ddelta
W(2,2) = 0.001; % penalty: a
W(3,3) = 1.000; % penalty: x
W(4,4) = 1.000; % penalty: y
W(5,5) = 0.100; % penalty: theta
W(6,6) = 0.001; % penalty: delta
W(7,7) = 0.001; % penality: v

% weight matrix in mayer term
W_e = W(nu+1:nu+nx, nu+1:nu+nx);


yr = [0;0;0;0;0;0;0]; % output reference in lagrange term
yr_e = yr(3:7); % output reference in mayer term
