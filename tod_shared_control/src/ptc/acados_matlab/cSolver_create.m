%% acados ocp model
ocp_model = acados_ocp_model();
ocp_model.set('name', model_name);
ocp_model.set('T', T);

% dims
ocp_model.set('dim_nx', nx);
ocp_model.set('dim_nu', nu);
if (strcmp(cost_type, 'linear_ls'))
    ocp_model.set('dim_ny', ny);
    ocp_model.set('dim_ny_e', ny_e);
end
ocp_model.set('dim_nbx', nbx);
ocp_model.set('dim_nbu', nbu);
ocp_model.set('dim_ng', ng);
ocp_model.set('dim_ng_e', ng_e);
ocp_model.set('dim_nh', nh);
ocp_model.set('dim_nh_e', nh_e);
% ocp_model.set('dim_ns', nsh);
% ocp_model.set('dim_nsh', nsh);

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
    ocp_model.set('cost_Vu', Vu);
    ocp_model.set('cost_Vx', Vx);
    ocp_model.set('cost_Vx_e', Vx_e);
    ocp_model.set('cost_W', W);
    ocp_model.set('cost_W_e', W_e);
    ocp_model.set('cost_y_ref', yr);
    ocp_model.set('cost_y_ref_e', yr_e);
%     ocp_model.set('cost_Z', Z);
%     ocp_model.set('cost_z', z);
elseif (strcmp(cost_type, 'ext_cost'))
    ocp_model.set('cost_expr_ext_cost', model.expr_ext_cost);
    ocp_model.set('cost_expr_ext_cost_e', model.expr_ext_cost_e);
end

% dynamics
if (strcmp(ocp_sim_method, 'erk'))
    ocp_model.set('dyn_type', 'explicit');
    ocp_model.set('dyn_expr_f', model.expr_f_expl);
else % irk
    ocp_model.set('dyn_type', 'implicit');
    ocp_model.set('dyn_expr_f', model.expr_f_impl);
end
% constraints
ocp_model.set('constr_x0', x0);
if (ng>0)
    ocp_model.set('constr_C', C);
    ocp_model.set('constr_D', D);
    ocp_model.set('constr_lg', lg);
    ocp_model.set('constr_ug', ug);
    ocp_model.set('constr_C_e', C_e);
    ocp_model.set('constr_lg_e', lg_e);
    ocp_model.set('constr_ug_e', ug_e);
elseif (nh>0)
    ocp_model.set('constr_expr_h', model.expr_h);
    ocp_model.set('constr_lh', lbu);
    ocp_model.set('constr_uh', ubu);
%     ocp_model.set('constr_Jsh', Jsh);
else
    ocp_model.set('constr_Jbx', Jbx);
    ocp_model.set('constr_lbx', lbx);
    ocp_model.set('constr_ubx', ubx);
    ocp_model.set('constr_Jbu', Jbu);
    ocp_model.set('constr_lbu', lbu);
    ocp_model.set('constr_ubu', ubu);
end

ocp_model.model_struct
