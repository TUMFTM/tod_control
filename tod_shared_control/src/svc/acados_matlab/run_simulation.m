% Simulation
close all;
N_sim = 1;
x_sim = zeros(model.nx, N_sim+1);
x_sim(:,1) = model.x0; % initial state
u_sim = zeros(model.nu, N_sim);
% set trajectory initialization
x_traj_init = [ ...
    linspace(model.x0(1), model.x0(1), ocp_N+1); ...
    linspace(model.x0(2), model.x0(1), ocp_N+1); ...
    linspace(model.x0(3), model.x0(1), ocp_N+1)];
u_traj_init = zeros(model.nu, ocp_N);
pi_traj_init = zeros(model.nx, ocp_N);

kappas = -0.45 * ones (1,ocp_N); %  .* exp(-0.2 * (1 : ocp_N));
s_safe = 5.0; 

% run
solve_times = zeros(1,N_sim);
sqp_iters = zeros(1,N_sim);
status_iter = zeros(1,N_sim);
for ii = 1 : N_sim
    curr_time = (ii-1) * h;
    % set x0
    ocp.set('constr_x0', x_sim(:,ii));
    
    % set trajectory initialization (if not, set internally using previous solution)
    ocp.set('init_x', x_traj_init);
    ocp.set('init_u', u_traj_init);
    ocp.set('init_pi', pi_traj_init);
    
    % curvature
    for jj = 0 : ocp_N-1
        params = [kappas(jj+1) , s_safe];
        ocp.set('p', params, jj);
    end

    tic;
    % solve OCP
    ocp.solve();
    dt = toc;
    solve_times(ii) = dt;
    
    status = ocp.get('status');
    sqp_iter = ocp.get('sqp_iter');
    time_tot = ocp.get('time_tot');
    time_lin = ocp.get('time_lin');
    time_qp_sol = ocp.get('time_qp_sol');
    fprintf('\nstatus = %d, sqp_iter = %d, time_int = %f [ms] (time_lin = %f [ms], time_qp_sol = %f [ms])\n',...
        status, sqp_iter, time_tot*1e3, time_lin*1e3, time_qp_sol*1e3);
    if status~=0
        disp('acados ocp solver failed');
        % break; % keyboard
    end
    status_iter(1,ii) = status;
    sqp_iters(1,ii) = sqp_iter;
    
    % get solution for initialization of next NLP
    x_traj = ocp.get('x');
    u_traj = ocp.get('u');
    pi_traj = ocp.get('pi');
    
    % shift trajectory for initialization
    x_traj_init = [x_traj(:,2:end), x_traj(:,end)];
    u_traj_init = [u_traj(:,2:end), u_traj(:,end)];
    pi_traj_init = [pi_traj(:,2:end), pi_traj(:,end)];
    
    % get solution for sim
    u_sim(:,ii) = ocp.get('u', 0);
    
    % set initial state of sim
    sim.set('x', x_sim(:,ii));
    % set input in sim
    sim.set('u', u_sim(:,ii));
    
    % simulate state
    sim.solve();
    
    % get new state
    x_sim(:,ii+1) = sim.get('xn');
end

figure; hold on; 
subplot(5,1,1); hold on; grid on; 
title('State'); 
plot(0:ocp_N, x_traj(1,:)); 
xlim([0,ocp_N+1]); 
ylabel('s in m'); hold off; 

subplot(5,1,2); hold on; grid on;  
title('Velocity');
plot(0:ocp_N, x_traj(2,:)); 
xlim([0,ocp_N+1]); 
ylabel('v in m/s'); hold off; 

subplot(5,1,3); hold on; grid on; 
title('Curvature'); plot(kappas); 
xlim([0,ocp_N+1]); 
ylabel('\kappa in 1/m'); 

subplot(5,1,4); hold on; grid on;  
title('Acceleration'); 
lat_accs = x_traj(2,2:end).^2 .* kappas; 
plot(0:ocp_N, x_traj(3,:)); 
plot(1:ocp_N, lat_accs); 
xlim([0,ocp_N+1]); 
ylabel('a in m/s^2'); 
legend('lon', 'lat'); 
xlabel('stage'); hold off; 

subplot(5,1,5); hold on; grid on, 
title('Jerk'), 
plot(0:ocp_N-1, u_traj(1,:)); 
xlim([0,ocp_N+1]); 
ylabel('j in m/s^3'); 
xlabel('stage'), hold off; 