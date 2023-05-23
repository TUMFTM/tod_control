function [sim_model,sim_opts] = create_sim_model(model, ocp_opts, ocp_N, T)
% acados sim model
sim_model = acados_sim_model();
% settings
sim_method = 'irk'; % erk, irk, irk_gnsf
sim_sens_forw = 'false'; % true, false
sim_num_stages = 4;
sim_num_steps = 4;
% dims
sim_model.set('dim_nx', model.nx);
sim_model.set('dim_nu', model.nu);
% symbolics
sim_model.set('sym_x', model.sym_x);
if isfield(model, 'sym_u')
    sim_model.set('sym_u', model.sym_u);
end
if isfield(model, 'sym_xdot')
    sim_model.set('sym_xdot', model.sym_xdot);
end
% model
sim_model.set('T', T/ocp_N);
if (strcmp(sim_method, 'erk'))
    sim_model.set('dyn_type', 'explicit');
    sim_model.set('dyn_expr_f', model.expr_f_expl);
else % irk
    sim_model.set('dyn_type', 'implicit');
    sim_model.set('dyn_expr_f', model.expr_f_impl);
end

%sim_model.model_struct


% acados sim opts
sim_opts = acados_sim_opts();
sim_opts.set('compile_interface', ocp_opts.opts_struct.compile_interface);
sim_opts.set('codgen_model', ocp_opts.opts_struct.codgen_model);
sim_opts.set('num_stages', sim_num_stages);
sim_opts.set('num_steps', sim_num_steps);
sim_opts.set('method', sim_method);
sim_opts.set('sens_forw', sim_sens_forw);

%sim_opts.opts_struct

end
