
mutable struct RelaxedIK_vars
    vars
    robot
    noise
    position_mode
    rotation_mode
    goal_positions
    goal_quats
    goal_positions_relative
    goal_quats_relative
    joint_goal
    init_ee_positions
    init_ee_quats
    joint_pts
    nn_model1
    nn_model2
    nn_model3
    w1
    nn_t1
    nn_c1
    nn_f1
    w2
    nn_t2
    nn_c2
    nn_f2
    w3
    nn_t3
    nn_c3
    nn_f3
    in_collision
    in_collision_groundtruth
    additional_vars
end

function RelaxedIK_vars(info, rcl_node, objectives, grad_types, weight_priors, inequality_constraints, ineq_grad_types, equality_constraints, eq_grad_types; position_mode = "relative", rotation_mode = "relative", preconfigured=false)
    lively_ik = pyimport("lively_ik")
    collision_transfer = pyimport("lively_ik.utils.collision_transfer")

    robot = yaml_block_to_robot(info)
    vars = Vars(info["starting_config"], objectives, grad_types, weight_priors, inequality_constraints, [], equality_constraints, [], info["joint_limits"])

    robot.getFrames(info["starting_config"])

    num_chains = robot.num_chains
    num_dc = length(info["joint_ordering"])

    goal_positions = []
    goal_quats = []
    goal_positions_relative = []
    goal_quats_relative = []
    init_ee_positions = Array{SArray{Tuple{3},Float64,1,3},1}()
    init_ee_quats = Array{Quat{Float64},1}()

    for i in 1:num_chains
        push!(init_ee_positions, robot.arms[i].out_pts[end])
        push!(init_ee_quats, Quat(robot.arms[i].out_frames[end]))
        push!(goal_positions, SVector(0.0,0.0,0.0))
        push!(goal_quats, Quat(1.,0.,0.,0.))
        push!(goal_positions_relative, SVector(0.,0.,0.))
        push!(goal_quats_relative, Quat(1.,0.,0.,0.))
    end

    arm_position_scale = zeros(num_chains)
    arm_rotation_scale = zeros(num_chains)
    dc_scale = zeros(num_dc)
    arm_position_freq = ones(num_chains)
    arm_rotation_freq = ones(num_chains)
    dc_freq = ones(num_dc)

    objectives = info["objectives"]

    noise = NoiseGenerator(objectives, info["fixed_frame_noise_scale"], info["fixed_frame_noise_frequency"])

    joint_goal = zeros(length(vars.init_state))

    if preconfigured == false
        robot_name = info["robot_name"]
        lively_ik_folder = lively_ik.BASE
        collision_nn_file = lively_ik_folder * "/config/collision_nn/" * robot_name
        w1 = BSON.load(collision_nn_file * "_1")[:w1]
        w1_ = Array{Array{Float64,2},1}()
        for i = 1:length(w1)
            push!(w1_, w1[i])
        end
        model1 = (x) -> predict(w1_, x)[1]
        w2 = BSON.load(collision_nn_file * "_2")[:w2]
        w2_ = Array{Array{Float64,2},1}()
        for i = 1:length(w2)
            push!(w2_, w2[i])
        end
        model2 = (x) -> predict(w2_, x)[1]
        w3 = BSON.load(collision_nn_file * "_3")[:w3]
        w3_ = Array{Array{Float64,2},1}()
        for i = 1:length(w3)
            push!(w3_, w3[i])
        end
        model3 = (x) -> predict(w3_, x)[1]

        fp = open(collision_nn_file * "_params_1", "r")
        nn_params_line = readline(fp)
        split_arr = split(nn_params_line, ",")
        t_val1 = parse(Float64, split_arr[1])
        c_val1 = parse(Float64, split_arr[2])
        f_val1 = parse(Float64, split_arr[3])
        close(fp)
        fp = open(collision_nn_file * "_params_2", "r")
        nn_params_line = readline(fp)
        split_arr = split(nn_params_line, ",")
        t_val2 = parse(Float64, split_arr[1])
        c_val2 = parse(Float64, split_arr[2])
        f_val2 = parse(Float64, split_arr[3])
        close(fp)
        fp = open(collision_nn_file * "_params_3", "r")
        nn_params_line = readline(fp)
        split_arr = split(nn_params_line, ",")
        t_val3 = parse(Float64, split_arr[1])
        c_val3 = parse(Float64, split_arr[2])
        f_val3 = parse(Float64, split_arr[3])
        close(fp)

        rv = RelaxedIK_vars(vars, robot, noise, position_mode, rotation_mode, goal_positions,
            goal_quats, goal_positions_relative, goal_quats_relative, joint_goal, init_ee_positions,
            init_ee_quats, 0, model1, model2, model3, w1_, t_val1, c_val1, f_val1, w2_, t_val2, c_val2, f_val2, w3_, t_val3, c_val3, f_val3, 0, 0, 0)
        initial_joint_points = state_to_joint_pts_withreturn(rand(length(vars.init_state)), rv)
        rv.joint_pts = initial_joint_points

        # state_to_joint_pts_closure = (x) -> state_to_joint_pts(x, rv)
        # collision_nn = (x)-> model_nn(x, model, state_to_joint_pts_closure)
        # rv.collision_nn = collision_nn
    else
        rv = RelaxedIK_vars(vars, robot, noise, position_mode, rotation_mode, goal_positions,
        goal_quats, goal_positions_relative, goal_quats_relative, joint_goal, init_ee_positions, init_ee_quats, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    end

    function in_collision(rv, x)
        state_to_joint_pts_inplace(x, rv)
        val = model2(rv.joint_pts)
        if val >= 1.0
            return true
        else
            return false
        end
    end

    in_collision_c = x->in_collision(rv, x)
    rv.in_collision = in_collision_c

    populate_vars!(vars, rv)

    if preconfigured == false
        for i = 1:length(grad_types)
            if grad_types[i] == "nn"
                # nn_grad_func_c = x->nn_grad_func(x, rv, rv.w, rv.nn_t, rv.nn_c, rv.nn_f)
                nn∇_1 = get_nn_grad_func(rv, rv.w1, rv.nn_t1, rv.nn_c1, rv.nn_f1)
                nn∇_2 = get_nn_grad_func(rv, rv.w2, rv.nn_t2, rv.nn_c2, rv.nn_f2)
                nn∇_3 = get_nn_grad_func(rv, rv.w3, rv.nn_t3, rv.nn_c3, rv.nn_f3)
                rv.vars.∇s[i] = nn∇_3
            end
        end

        cv = collision_transfer.CollisionVars(info,rcl_node)

        function in_collision_groundtruth(cv, x)
            score = collision_transfer.get_score(x, cv)
            if score >= 5.0
                return true
            else
                return false
            end
        end

        in_collision_groundtruth_c = x->in_collision_groundtruth(cv, x)
        rv.in_collision_groundtruth = in_collision_groundtruth_c
    end

    return rv
end

function update_relaxedIK_vars!(relaxedIK_vars, xopt)
    update!(relaxedIK_vars.vars, xopt)
end

function yaml_block_to_robot(info)
    arms = yaml_block_to_arms(info)
    robot = Robot(arms, info["joint_names"], info["joint_ordering"], info["joint_limits"], info["velocity_limits"])
    return robot
end

function yaml_block_to_arms(info)
    num_chains = length(info["joint_names"])

    arms = []

    for i=1:num_chains
        a = Arm(info["joint_names"][i], info["axis_types"][i], info["displacements"][i],  info["disp_offsets"][i],
            info["rot_offsets"][i], info["joint_types"][i], true)
        push!(arms, a)
    end

    return arms
end

function get_nn_grad_func(relaxedIK_vars, w, nn_t, nn_c, nn_f)
    function nn_grad_func(x)
        ∇, nn_output = get_gradient_wrt_input(w, relaxedIK_vars.joint_pts)
        get_linear_jacobian(relaxedIK_vars.robot, x)
        jac = relaxedIK_vars.robot.linear_jacobian
        gld = groove_loss_derivative(nn_output[1], nn_t, 2, nn_c, nn_f, 2)
        ∇ = gld*∇*jac
        ret = Array{Float64, 1}()
        for i in ∇
            push!(ret, i)
        end
        return ret
    end
    return nn_grad_func
end
