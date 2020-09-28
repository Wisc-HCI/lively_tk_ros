
mutable struct Goals
    positions#::Array{Array{Float64}}
    quats    #::Array{Array{Float64}}
    dc       #::Array{Float64}
    time     ::Float64
    bias     #::Array{Float64}
    weights  #::Array{Float64}
    metadata
end

function update!(goal,msg,time::Float64)
    positions = []
    quats = []

    # Create POS/QUAT Goals from EE Poses
    for i = 1:length(msg.ee_poses)
        p = msg.ee_poses[i]

        pos_x = p.position.x
        pos_y = p.position.y
        pos_z = p.position.z

        quat_w = p.orientation.w
        quat_x = p.orientation.x
        quat_y = p.orientation.y
        quat_z = p.orientation.z

        push!(positions, [pos_x, pos_y, pos_z])
        push!(quats, Quat(quat_w, quat_x, quat_y, quat_z))
    end
    goal.positions = positions
    goal.quats = quats

    # Create DC Goals from DC Values
    dc = []
    for i=1:length(msg.dc_values)
        push!(dc,msg.dc_values[i].data)
    end
    goal.dc = dc

    # Update time
    goal.time = time

    # Extract Bias
    goal.bias = [msg.bias.x,msg.bias.y,msg.bias.z]

    # Extract Weights
    weights = []
    for i=1:length(msg.objective_weights)
        push!(weights,msg.objective_weights[i].data)
    end
    goal.weights = weights
    goal.metadata = msg.metadata.data
end

function Goals(lively_ik,info_data)
    num_chains = lively_ik.relaxedIK_vars.robot.num_chains

    positions = []
    quats = []

    # Create POS/QUAT Goals from EE Poses
    for i = 1:length(num_chains)
        push!(positions, lively_ik.relaxedIK_vars.init_ee_positions[i])
        push!(quats, lively_ik.relaxedIK_vars.init_ee_quats[i])
    end

    # Create DC Goals from DC Values
    dc = info_data["starting_config"]

    # Get Time from Header
    time = 0.0

    # Extract Bias
    bias = [1.0,1.0,1.0]

    # Extract Weights
    weights = lively_ik.relaxedIK_vars.vars.weight_priors

    return Goals(positions, quats, dc, time, bias, weights, "")
end
