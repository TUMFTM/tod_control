function ocp_model = create_ocp_model(model, ocp_opts, T, cost_type)
% acados ocp model
ocp_model = acados_ocp_model();
ocp_model.set('name', model.name);
ocp_model.set('T', T);

% dims
ocp_model.set('dim_nx', model.nx);
ocp_model.set('dim_nu', model.nu);
if (strcmp(cost_type, 'linear_ls'))
    ocp_model.set('dim_ny', model.ny);
    ocp_model.set('dim_ny_e', model.ny_e);
end
ocp_model.set('dim_nbx', model.nbx);
ocp_model.set('dim_nbu', model.nbu);
ocp_model.set('dim_ng', model.ng);
ocp_model.set('dim_ng_e', model.ng_e);
ocp_model.set('dim_nh', model.nh);
ocp_model.set('dim_nh_e', model.nh_e);
ocp_model.set('dim_ns', model.nsh);
ocp_model.set('dim_nsh', model.nsh);

% symbolics
ocp_model.set('sym_x', model.sym_x);
if isfield(model, 'sym_u')
    ocp_model.set('sym_u', model.sym_u);
end
if isfield(model, 'sym_xdot')
    ocp_model.set('sym_xdot', model.sym_xdot);
end
if isfield(model, 'np')
    ocp_model.set('dim_np', model.np);
    ocp_model.set('sym_p', model.sym_p);
end

% cost
ocp_model.set('cost_type', cost_type);
ocp_model.set('cost_type_e', cost_type);
if (strcmp(cost_type, 'linear_ls'))
    ocp_model.set('cost_Vu', model.Vu);
    ocp_model.set('cost_Vx', model.Vx);
    ocp_model.set('cost_Vx_e', model.Vx_e);
    ocp_model.set('cost_W', model.W);
    ocp_model.set('cost_W_e', model.W_e);
    ocp_model.set('cost_y_ref', model.yr);
    ocp_model.set('cost_y_ref_e', model.yr_e);
    ocp_model.set('cost_Z', model.Z);
    ocp_model.set('cost_z', model.z);
elseif (strcmp(cost_type, 'ext_cost'))
    ocp_model.set('cost_expr_ext_cost', model.expr_ext_cost);
    ocp_model.set('cost_expr_ext_cost_e', model.expr_ext_cost_e);
end

% dynamics
if (strcmp(ocp_opts.opts_struct.sim_method, 'erk'))
    ocp_model.set('dyn_type', 'explicit');
    ocp_model.set('dyn_expr_f', model.expr_f_expl);
else % irk
    ocp_model.set('dyn_type', 'implicit');
    ocp_model.set('dyn_expr_f', model.expr_f_impl);
end
% constraints
ocp_model.set('constr_x0', model.x0);
ocp_model.set('constr_expr_h', model.expr_h);
ocp_model.set('constr_lh', model.lbu);
ocp_model.set('constr_uh', model.ubu);
ocp_model.set('constr_Jsh', model.Jsh);

% ocp_model.model_struct

end
