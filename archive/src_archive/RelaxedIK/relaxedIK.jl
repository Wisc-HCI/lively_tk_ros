


include("GROOVE_RelaxedIK_Julia/relaxedIK_objective.jl")
include("GROOVE_RelaxedIK_Julia/livelyIK_objective.jl")
include("GROOVE_RelaxedIK_Julia/relaxedIK_vars.jl")
include("GROOVE_Julia/groove.jl")
include("GROOVE_RelaxedIK_Julia/relaxedIK_objective.jl")
include("Utils_Julia/transformations.jl")
include("Utils_Julia/ema_filter.jl")

mutable struct RelaxedIK
    relaxedIK_vars
    groove
    ema_filter
end


function RelaxedIK(path_to_src, info_file_name, objectives, grad_types, weight_priors, inequality_constraints, ineq_grad_types, equality_constraints, eq_grad_types; position_mode = "absolute", rotation_mode = "absolute", solver_name="slsqp", preconfigured=false, groove_iter = 11, max_time=0.0)
    relaxedIK_vars = RelaxedIK_vars(path_to_src, info_file_name, objectives, grad_types, weight_priors, inequality_constraints, ineq_grad_types, equality_constraints, eq_grad_types, position_mode = position_mode, rotation_mode = rotation_mode, preconfigured=preconfigured)
    groove = get_groove(relaxedIK_vars.vars, solver_name, max_iter = groove_iter, max_time=max_time)
    ema_filter = EMA_filter(relaxedIK_vars.vars.init_state)
    return RelaxedIK(relaxedIK_vars, groove, ema_filter)
end

function get_standard(path_to_src, info_file_name; solver_name = "slsqp", preconfigured=false)
    y = info_file_name_to_yaml_block(path_to_src, info_file_name)
    # ee_position_weight = y["ee_position_weight"]
    # ee_rotation_weight = y["ee_rotation_weight"]
    # dc_joint_weight = y["dc_joint_weight"]
    joint_ordering = y["joint_ordering"]
    joint_names = y["joint_names"]
    # match_objectives = y["match_objectives"]
    objective_info = y["objectives"]
    objectives =    []
    grad_types =    []
    weight_priors = []

    for i in 1:length(objective_info)
        if objective_info[i]["type"] == "min_jt_vel"
            push!(objectives, min_jt_vel_obj)
            push!(grad_types, objective_info[i]["gradient"])
            push!(weight_priors, objective_info[i]["weight"])
        elseif objective_info[i]["type"] == "min_jt_accel"
            push!(objectives, min_jt_accel_obj)
            push!(grad_types, objective_info[i]["gradient"])
            push!(weight_priors, objective_info[i]["weight"])
        elseif objective_info[i]["type"] == "min_jt_jerk"
            push!(objectives, min_jt_jerk_obj)
            push!(grad_types, objective_info[i]["gradient"])
            push!(weight_priors, objective_info[i]["weight"])
        elseif objective_info[i]["type"] == "joint_limit"
            push!(objectives, joint_limit_obj)
            push!(grad_types, objective_info[i]["gradient"])
            push!(weight_priors, objective_info[i]["weight"])
        elseif objective_info[i]["type"] == "collision_nn"
            push!(objectives, collision_nn_obj)
            push!(grad_types, objective_info[i]["gradient"])
            push!(weight_priors, objective_info[i]["weight"])
        elseif objective_info[i]["type"] == "position"
            push!(objectives,(x,vars)->position_obj(x,vars,objective_info[i]["index"]))
            push!(grad_types, objective_info[i]["gradient"])
            push!(weight_priors, objective_info[i]["weight"])
        elseif objective_info[i]["type"] == "positional_noise"
            #println("adding pos obj $i")
            push!(objectives,(x,vars)->positional_noise_obj(x,vars,i,objective_info[i]["index"]))
            push!(grad_types, objective_info[i]["gradient"])
            push!(weight_priors, objective_info[i]["weight"])
        elseif objective_info[i]["type"] == "rotation"
            push!(objectives,(x,vars)->rotation_obj(x,vars,objective_info[i]["index"]))
            push!(grad_types, objective_info[i]["gradient"])
            push!(weight_priors, objective_info[i]["weight"])
        elseif objective_info[i]["type"] == "rotational_noise"
            #println("adding rot obj $i")
            push!(objectives,(x,vars)->rotational_noise_obj(x,vars,i,objective_info[i]["index"]))
            push!(grad_types, objective_info[i]["gradient"])
            push!(weight_priors, objective_info[i]["weight"])
        elseif objective_info[i]["type"] == "dc"
            push!(objectives,(x,vars)->dc_obj(x,vars,objective_info[i]["index"]))
            push!(grad_types, objective_info[i]["gradient"])
            push!(weight_priors, objective_info[i]["weight"])
        elseif objective_info[i]["type"] == "dc_noise"
            #println("adding dc obj $i")
            push!(objectives,(x,vars)->dc_noise_obj(x,vars,i,objective_info[i]["index"]))
            push!(grad_types, objective_info[i]["gradient"])
            push!(weight_priors, objective_info[i]["weight"])
        elseif objective_info[i]["type"] == "x_match_position"
            push!(objectives,(x,vars)->x_match_obj(x,vars,objective_info[i]["index_1"],objective_info[i]["index_2"]))
            push!(grad_types, objective_info[i]["gradient"])
            push!(weight_priors, objective_info[i]["weight"])
        elseif objective_info[i]["type"] == "y_match_position"
            push!(objectives,(x,vars)->y_match_obj(x,vars,objective_info[i]["index_1"],objective_info[i]["index_2"]))
            push!(grad_types, objective_info[i]["gradient"])
            push!(weight_priors, objective_info[i]["weight"])
        elseif objective_info[i]["type"] == "z_match_position"
            push!(objectives,(x,vars)->x_match_obj(x,vars,objective_info[i]["index_1"],objective_info[i]["index_2"]))
            push!(grad_types, objective_info[i]["gradient"])
            push!(weight_priors, objective_info[i]["weight"])
        elseif objective_info[i]["type"] == "orientation_match"
            push!(objectives,(x,vars)->orientation_match_obj(x,vars,objective_info[i]["index_1"],objective_info[i]["index_2"]))
            push!(grad_types, objective_info[i]["gradient"])
            push!(weight_priors, objective_info[i]["weight"])
        elseif objective_info[i]["type"] == "joint_match"
            push!(objectives,(x,vars)->joint_match_obj(x,vars,objective_info[i]["index_1"],objective_info[i]["index_2"]))
            push!(grad_types, objective_info[i]["gradient"])
            push!(weight_priors, objective_info[i]["weight"])
        end

    end

    inequality_constraints = []
    ineq_grad_types = []
    equality_constraints = []
    eq_grad_types = []
    return RelaxedIK(path_to_src, info_file_name, objectives, grad_types, weight_priors, inequality_constraints, ineq_grad_types, equality_constraints, eq_grad_types, solver_name = solver_name, preconfigured=preconfigured)
end

function get_nchain_base_ik(n, path_to_src, info_file_name; solver_name = "slsqp", preconfigured=false)
    objectives = []
    grad_types = []
    weight_priors = []
    for i in 1:n
        # Add position objective
        push!(objectives,(x,vars)->position_obj(x,vars,i))
        push!(grad_types,"forward_ad")
        push!(weight_priors,ee_position_weight[i])

        # Add orientation objective
        push!(objectives,(x,vars)->rotation_obj(x,vars,i))
        push!(grad_types,"forward_ad")
        push!(weight_priors,ee_rotation_weight[i])
    end
    inequality_constraints = []
    ineq_grad_types = []
    equality_constraints = []
    eq_grad_types = []
    return RelaxedIK(path_to_src, info_file_name, objectives, grad_types, weight_priors, inequality_constraints, ineq_grad_types, equality_constraints, eq_grad_types, solver_name = solver_name, preconfigured=preconfigured)
end


function get_base_ik(path_to_src, info_file_name; solver_name = "slsqp", preconfigured=false)
    objectives = [position_obj_1, rotation_obj_1]
    grad_types = ["forward_ad", "forward_ad"]
    weight_priors = [1.0, 1.0]
    inequality_constraints = []
    ineq_grad_types = []
    equality_constraints = []
    eq_grad_types = []
    relaxedIK = RelaxedIK(path_to_src, info_file_name, objectives, grad_types, weight_priors, inequality_constraints, ineq_grad_types, equality_constraints, eq_grad_types, solver_name = solver_name, preconfigured=preconfigured, groove_iter=60, max_time = 0.0023)
    num_chains = relaxedIK.relaxedIK_vars.robot.num_chains
    if num_chains > 1
        relaxedIK = get_nchain_base_ik(num_chains, path_to_src, info_file_name)
    end
    return relaxedIK
end

function get_finite_diff_version(path_to_src, info_file_name; solver_name = "slsqp", preconfigured=false)
    objectives = [position_obj_1, rotation_obj_1, min_jt_vel_obj, min_jt_accel_obj, min_jt_jerk_obj, collision_nn_obj]
    grad_types = ["finite_diff", "finite_diff", "finite_diff", "finite_diff", "finite_diff",         "finite_diff"]
    weight_priors = [50., 40.0, 1.0 ,1.0, 1.0, 0.4]
    inequality_constraints = []
    ineq_grad_types = []
    equality_constraints = []
    eq_grad_types = []
    return RelaxedIK(path_to_src, info_file_name, objectives, grad_types, weight_priors, inequality_constraints, ineq_grad_types, equality_constraints, eq_grad_types, solver_name = solver_name, preconfigured=preconfigured)
end

function get_joint_positions(relaxedIK, x)
    positions = []
    for i = 1:length(relaxedIK.relaxedIK_vars.robot.subchain_indices)
        push!(positions, [])
        relaxedIK.relaxedIK_vars.robot.arms[i].getFrames(x[relaxedIK.relaxedIK_vars.robot.subchain_indices[i]])
        for j = 1:length(relaxedIK.relaxedIK_vars.robot.arms[i].out_pts)
            push!(positions[i], relaxedIK.relaxedIK_vars.robot.arms[i].out_pts[j])
        end
    end
    return positions
end

function get_rot_mats(relaxedIK, x)
    rot_mats = []
    for i = 1:length(relaxedIK.relaxedIK_vars.robot.subchain_indices)
        push!(rot_mats, [])
        relaxedIK.relaxedIK_vars.robot.arms[i].getFrames(x[relaxedIK.relaxedIK_vars.robot.subchain_indices[i]])
        for j = 1:length(relaxedIK.relaxedIK_vars.robot.arms[i].out_frames)
            push!(rot_mats[i], relaxedIK.relaxedIK_vars.robot.arms[i].out_frames[j])
        end
    end
    return rot_mats
end

function solve(relaxedIK, goal_positions, goal_quats, dc_goals, time, bias, weights; prev_state = nothing, filter=true, max_iter = 0, max_time = 0.05)
    vars = relaxedIK.relaxedIK_vars
    vars.vars.weight_priors = weights

    if vars.position_mode == "relative"
        vars.goal_positions = copy(vars.init_ee_positions)
        for i = 1:vars.robot.num_chains
            vars.goal_positions[i] += goal_positions[i]
        end
    else
        vars.goal_positions = goal_positions
    end

    update!(vars.noise, time, bias)

    # Assign the dc_goals to the joint_goal in vars
    vars.joint_goal = dc_goals

    if vars.rotation_mode == "relative"
        for i = 1:vars.robot.num_chains
            vars.goal_quats[i] = goal_quats[i] * copy(vars.init_ee_quats[i])
        end
    else
        vars.goal_quats = goal_quats
    end

    xopt = groove_solve(relaxedIK.groove, prev_state=prev_state, max_iter=max_iter, max_time = max_time)
    update_relaxedIK_vars!(relaxedIK.relaxedIK_vars, xopt)
    if filter
        xopt = filter_signal(relaxedIK.ema_filter, xopt)
    end

    return xopt
end

function solve_precise(relaxedIK, goal_positions, goal_quats, dc_goals, time, bias, weights; prev_state = nothing, pos_tol = 0.001, rot_tol = 0.001, max_tries = 3, max_iter = 0, max_time = 0.0)
    xopt = solve(relaxedIK, goal_positions, goal_quats, dc_goals, time, bias, weights, prev_state = prev_state, filter = false, max_iter = max_iter, max_time = 0.0)
    valid_sol = true
    pos_error = 0.0
    rot_error = 0.0

    for i = 1:length(goal_positions)
        pos_error, rot_error = get_ee_error(relaxedIK, xopt, goal_positions[1], goal_quats[1], i)
        if (pos_error > pos_tol || rot_error > rot_tol)
            valid_sol = false
        end
    end

    try_idx = 1
    while (! valid_sol) && (try_idx < max_tries)
        xopt = solve(relaxedIK, goal_positions, goal_quats, dc_goals, time, bias, weights, prev_state = xopt, filter = false)
        valid_sol = true
        for i = 1:length(goal_positions)
            pos_error, rot_error = get_ee_error(relaxedIK, xopt, goal_positions[1], goal_quats[1], i)
            if (pos_error > pos_tol || rot_error > rot_tol)
                valid_sol = false
            end
        end
        try_idx += 1
    end

    return xopt, try_idx, valid_sol, pos_error, rot_error
end

function get_ee_error(relaxedIK, xopt, goal_pos, goal_quat, armidx)

    vars = relaxedIK.relaxedIK_vars

    if vars.position_mode == "relative"
        goal_pos += vars.init_ee_positions[armidx]
    end


    if vars.rotation_mode == "relative"
        goal_quat = goal_quat * copy(vars.init_ee_quats[armidx])
    end

    arm = relaxedIK.relaxedIK_vars.robot.arms[armidx]
    arm.getFrames(xopt)
    ee_pos = arm.out_pts[end]
    ee_quat = Quat(arm.out_frames[end])

    pos_error = norm(ee_pos - goal_pos)
    rot_error = norm( quaternion_disp(ee_quat, goal_quat) )

    return pos_error, rot_error
end

function in_collision(relaxedIK, x)
    return relaxedIK.relaxedIK_vars.in_collision(x)
end

function in_collision_groundtruth(relaxedIK, x)
    return relaxedIK.relaxedIK_vars.in_collision_groundtruth(x)
end