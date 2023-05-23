function plotXY(x_sim, model, x_traj)


figure
hold on;
plot(x_sim(1,:),x_sim(2,:));

for i = 1 : length(model.xo_ell)
    xposel = model.xo_ell(i);
    yposel = model.yo_ell(i);
    aell = model.a_ell(i);
    bell = model.b_ell(i);
    phiell = model.phi_ell(i); 
    nell = model.n_ell;
    rad = 0; % model.radius; 
    fimplicit(@(x,y) (((x-xposel)*cos(phiell)-(y-yposel)*sin(phiell)) ./ ...
        (aell+rad)).^nell + ...
        (((y-yposel)*cos(phiell)+(x-xposel)*sin(phiell)) ./ ...
        (bell+rad)).^nell - 1, ...
        [-1000,1000], 'r', 'linewidth', 2);
end

if length(x_traj)
    plot(x_traj(1,:), x_traj(2,:), 'g'); 
end

xlim([0,250]);
ylim([0,60]);
grid on; hold off;

end