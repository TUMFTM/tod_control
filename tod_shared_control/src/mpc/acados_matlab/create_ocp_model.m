function ocp_model = create_ocp_model(mdl, ocp_opts, T, cost_type)
% acados ocp model
ocp_model = acados_ocp_model();
ocp_model.set('name', mdl.name);
ocp_model.set('T', T);

% dims
ocp_model.set('dim_nx', mdl.nx);
ocp_model.set('dim_nu', mdl.nu);
if (strcmp(cost_type, 'linear_ls'))
    ocp_model.set('dim_ny', mdl.ny);
    ocp_model.set('dim_ny_e', mdl.ny_e);
end
ocp_model.set('dim_nbx', mdl.nbx);
ocp_model.set('dim_nbu', mdl.nbu);
ocp_model.set('dim_ng', mdl.ng);
ocp_model.set('dim_ng_e', mdl.ng_e);
ocp_model.set('dim_nh', mdl.nh);
ocp_model.set('dim_nh_e', mdl.nh_e);
ocp_model.set('dim_ns', mdl.nsh);
ocp_model.set('dim_nsh', mdl.nsh);

% symbolics
ocp_model.set('sym_x', mdl.sym_x);
if isfield(mdl, 'sym_u')
    ocp_model.set('sym_u', mdl.sym_u);
end
if isfield(mdl, 'sym_xdot')
    ocp_model.set('sym_xdot', mdl.sym_xdot);
end
if isfield(mdl, 'np')
    ocp_model.set('dim_np', mdl.np);
    ocp_model.set('sym_p', mdl.sym_p);
end

% cost
ocp_model.set('cost_type', cost_type);
ocp_model.set('cost_type_e', cost_type);
if (strcmp(cost_type, 'linear_ls'))
    ocp_model.set('cost_Vu', mdl.Vu);
    ocp_model.set('cost_Vx', mdl.Vx);
    ocp_model.set('cost_Vx_e', mdl.Vx_e);
    ocp_model.set('cost_W', mdl.W);
    ocp_model.set('cost_W_e', mdl.W_e);
    ocp_model.set('cost_y_ref', mdl.yr);
    ocp_model.set('cost_y_ref_e', mdl.yr_e);
    ocp_model.set('cost_Z', mdl.Z);
    ocp_model.set('cost_z', mdl.z);
elseif (strcmp(cost_type, 'ext_cost'))
    ocp_model.set('cost_expr_ext_cost', model.expr_ext_cost);
    ocp_model.set('cost_expr_ext_cost_e', model.expr_ext_cost_e);
end

% dynamics
if (strcmp(ocp_opts.opts_struct.sim_method, 'erk'))
    ocp_model.set('dyn_type', 'explicit');
    ocp_model.set('dyn_expr_f', mdl.expr_f_expl);
else % irk
    ocp_model.set('dyn_type', 'implicit');
    ocp_model.set('dyn_expr_f', mdl.expr_f_impl);
end
% constraints
ocp_model.set('constr_x0', mdl.x0);
if (mdl.ng>0)
    ocp_model.set('constr_C', C);
    ocp_model.set('constr_D', D);
    ocp_model.set('constr_lg', lg);
    ocp_model.set('constr_ug', ug);
    ocp_model.set('constr_C_e', C_e);
    ocp_model.set('constr_lg_e', lg_e);
    ocp_model.set('constr_ug_e', ug_e);
elseif (mdl.nh>0)
    ocp_model.set('constr_expr_h', mdl.expr_h);
    ocp_model.set('constr_lh', mdl.lbu);
    ocp_model.set('constr_uh', mdl.ubu);
    ocp_model.set('constr_Jsh', mdl.Jsh);
else
    ocp_model.set('constr_Jbx', Jbx);
    ocp_model.set('constr_lbx', lbx);
    ocp_model.set('constr_ubx', ubx);
    ocp_model.set('constr_Jbu', Jbu);
    ocp_model.set('constr_lbu', lbu);
    ocp_model.set('constr_ubu', ubu);
end

ocp_model.model_struct
end
