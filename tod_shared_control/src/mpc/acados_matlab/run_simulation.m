% Simulation
close all;
x0 = mdl.x0; 
N_sim = 75.0 / h; % model1500;
x_sim = zeros(mdl.nx, N_sim+1);
x_sim(:,1) = x0; % initial state
u_sim = zeros(mdl.nu, N_sim);
% set trajectory initialization
x_traj_init = [ ...
    linspace(x0(1), x0(1), ocp_N+1); ...
    linspace(x0(2), x0(1), ocp_N+1); ...
    linspace(x0(3), x0(3), ocp_N+1); ...
    linspace(x0(4), x0(4), ocp_N+1); ...
    linspace(x0(5), x0(5), ocp_N+1)];
u_traj_init = zeros(mdl.nu, ocp_N);
pi_traj_init = zeros(mdl.nx, ocp_N);

% run
solve_times = zeros(1,N_sim);
sqp_iters = zeros(1,N_sim);
status_iter = zeros(1,N_sim);
delta_ref_iter = zeros(1,N_sim);
slacks_max_lower = zeros(1,N_sim); 
slacks_max_uppers = zeros(1,N_sim);
for ii = 1 : N_sim
    
    curr_time = (ii-1) * h;
    delta_ref = deg2rad(2.5) * sin(2*pi*curr_time/20);
    delta_ref_iter(ii) = delta_ref;
    
    % set x0
    ocp.set('constr_x0', x_sim(:,ii));
    
    % vary reference
    %     yr(4) = yr(4)-0.001;
    mdl.yr(6) = delta_ref;
    %     yr(7) = yr(7) - 0.001;
    mdl.yr_e = mdl.yr(3:7); % output reference in mayer term
    ocp.set('cost_y_ref', mdl.yr);
    ocp.set('cost_y_ref_e', mdl.yr_e);
    
    p_vec = [];
    for i = 1 : length(mdl.xo_ell)
        p_vec = [p_vec , mdl.xo_ell(i) , mdl.yo_ell(i) , ...
            mdl.a_ell(i) , mdl.b_ell(i) , mdl.phi_ell(i)];
    end
    ocp.set('p', p_vec);
    
    % set trajectory initialization (if not, set internally using previous solution)
    ocp.set('init_x', x_traj_init);
    ocp.set('init_u', u_traj_init);
    ocp.set('init_pi', pi_traj_init);
    
%     % use ocp.set to modify numerical data for a certain stage
%     some_stages = 1:10:ocp_N-1;
%     for i = some_stages
%         if strcmp( ocp.model_struct.cost_type, 'linear_ls')
%             ocp.set('cost_Vx', Vx, i);
%         end
%     end
    
    tic;
    % solve OCP
    ocp.solve();
    dt = toc;
    solve_times(ii) = dt;
    
    if 1
        status = ocp.get('status');
        sqp_iter = ocp.get('sqp_iter');
        time_tot = ocp.get('time_tot');
        time_lin = ocp.get('time_lin');
        time_qp_sol = ocp.get('time_qp_sol');
        slacksLower = zeros(mdl.nsh,ocp_N);
        slacksUpper = zeros(mdl.nsh,ocp_N);
        for stage = 0 : ocp_N-1
            slacksLower(:, stage+1) = ocp.get('sl',stage);
            slacksUpper(:, stage+1) = ocp.get('su',stage);
        end
        statePredictions = ocp.get('x');
        slacksMaxLower(ii) = max(max(abs(slacksLower))); 
        slacksMaxUpper(ii) = max(max(abs(slacksUpper))); 
        
        if false
            %         if sqp_iter > 4 && false
            % residuals processing
            stats = ocp.get('stat');
            ocp.print('stat');
            close all;
            %             figure;  hold on;
            %             for i = 2 : 5
            %                 plot(stats(:,1), stats(:,i));
            %             end
            %             legend('stat', 'eq', 'ineq', 'comp');
            %             grid on;
            %             xlabel('sqp iter'); ylabel('res');
            %             ylim([0.0, 20*myNLPtolerance]);
            %             hold off;
            
            max(max(abs(slacksLower)))
            max(max(abs(slacksUpper)))
            figure; hold on;
            subplot(2,1,1); hold on;
            for var = 1 : size(slacksLower,1)
                plot(slacksLower(var,:));
            end
            legend('deref','ox1','oy1','ox2','oy2','ox3','oy3');
            xlabel('stage'); ylabel('slack');
            subplot(2,1,2); hold on;
            for var = 1 : size(slacksUpper,1)
                plot(slacksUpper(var,:));
            end
            xlabel('stage'); ylabel('slack');
            legend('deref','ox1','oy1','ox2','oy2','ox3','oy3');
            hold off;
            % ocp.print('stat');
        end
        
        fprintf('\nstatus = %d, sqp_iter = %d, time_int = %f [ms] (time_lin = %f [ms], time_qp_sol = %f [ms])\n',...
            status, sqp_iter, time_tot*1e3, time_lin*1e3, time_qp_sol*1e3);
        if status~=0
            disp('acados ocp solver failed');
            %             break; % keyboard
        end
        status_iter(1,ii) = status;
        sqp_iters(1,ii) = sqp_iter;
    end
    
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
    
    intervene = abs(delta_ref - x_sim(4,ii));
    if intervene > 0.01 && false
        plotXY(x_sim(:,1:ii), model, x_traj);
        plotTimeCourses(x_sim(:,1:ii), u_sim(:,1:ii), delta_ref_iter(1:ii), delta_dev_max, x_traj, u_traj);
        close all;
    end
    
    % set initial state of sim
    sim.set('x', x_sim(:,ii));
    % set input in sim
    sim.set('u', u_sim(:,ii));
    
    % simulate state
    sim.solve();
    
    % get new state
    x_sim(:,ii+1) = sim.get('xn');
end
close all;

% figure
% hold on; 
% plot(slacksMaxLower); 
% plot(slacksMaxUpper); 
% legend('lower', 'upper'); 
% hold off;