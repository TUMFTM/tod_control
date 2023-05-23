
close all;
plot_xy(x_sim, mdl, []); 
plot_states_over_time(x_sim, u_sim, delta_ref_iter, [], []); 


figure
solve_timesPLOT = solve_times*1000;
subplot(2,1,1); hold on;
plot(solve_timesPLOT);
plot([1,length(solve_timesPLOT)],[mean(solve_timesPLOT),mean(solve_timesPLOT)]);
title('Solving Times in ms');
grid on; hold off;

subplot(2,1,2); hold on;
plot(sqp_iters); % ,'rx');
plot(status_iter,'rx');
title('Number of SQP Iterations');
%     xlabel('Time Step');
ylabel('n');
%     ylim([-1,11]);
legend('sqp iter', 'status');
grid on; hold off;


clc;
disp(['dt = ', num2str(h), ' - NlpTol = ', num2str(ocp_opts.opts_struct.nlp_solver_tol_eq), ...
    ' - QpTol = ', num2str(ocp_opts.opts_struct.qp_solver_tol_eq), ...
    ' - MaxSqpIter = ', num2str(max(sqp_iters)), ' - Solver Times: Mean = ', ...
    num2str(mean(solve_timesPLOT)), ' ms - Max = ', num2str(max(solve_timesPLOT)), ' ms']);
