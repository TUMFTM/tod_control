function plotTimeCourses(x_sim, u_sim, delta_ref_iter, x_traj, u_traj)

figure
subplot(4,1,1); hold on;
title('Delta in deg');
plot(rad2deg(x_sim(4,:)), 'b');
plot(rad2deg(delta_ref_iter),'g--');
if ~isempty(x_traj)
    plot([1 : length(x_traj(4,:))] + length(x_sim(4,:)) - 1, rad2deg(x_traj(4,:)), 'm');
end
grid on; hold off;

subplot(4,1,2); hold on;
title('DDelta in deg/s');
plot(rad2deg(u_sim(1,:)));
if ~isempty(u_traj)
    plot([1 : length(u_traj(1,:))-1] + length(u_sim(1,:)) - 1, rad2deg(u_traj(1,1:end-1)), 'm');
end
grid on; hold off;

subplot(4,1,3); hold on;
title('Velocity in m/s');
plot(x_sim(5,:));
if ~isempty(x_traj)
    plot([1 : length(x_traj(5,:))] + length(x_sim(5,:)) - 1, x_traj(5,:), 'm');
end
grid on; hold off;

subplot(4,1,4); hold on;
title('Acceleration in m/s^2');
plot(u_sim(2,:));
if ~isempty(u_traj)
    plot([1 : length(u_traj(2,:))-1] + length(u_sim(2,:)) - 1, u_traj(2,1:end-1), 'm');
end
grid on; hold off;

end

