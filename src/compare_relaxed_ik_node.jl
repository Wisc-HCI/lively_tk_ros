#!/usr/bin/env julia
include("RelaxedIK/relaxedIK.jl")
include("RelaxedIK/GROOVE_RelaxedIK_Julia/relaxedIK_vars.jl")
include("RelaxedIK/Utils_Julia/solver_output.jl")
using YAML
using RobotOS
using Rotations
using ForwardDiff

@rosimport wisc_msgs.msg: DCPoseGoals, EEPoseGoals, JointAngles, DebugGoals, DebugPoseAngles
@rosimport std_msgs.msg: Float64MultiArray, Bool, Float32
@rosimport geometry_msgs.msg: Point, Quaternion, Pose

rostypegen()
using .wisc_msgs.msg
using .std_msgs.msg
using .geometry_msgs.msg

global goal_info = nothing
global solution = nothing

function set_goal_info(data)
    global goal_info = data
end

init_node("lively_ik_node")


global angles_pub = Publisher("/relaxed_ik/debug_pose_angles", DebugPoseAngles, queue_size = 3)

path_to_src = Base.source_dir()

loaded_robot_file = open(path_to_src * "/RelaxedIK/Config/loaded_robot")
loaded_robot = readline(loaded_robot_file)
close(loaded_robot_file)
relaxedIK = get_standard(path_to_src, loaded_robot)
livelyIK = get_standard(path_to_src, loaded_robot)
num_chains = relaxedIK.relaxedIK_vars.robot.num_chains

println("loaded robot: $loaded_robot")

objectives = get_param("objectives")
starting_config = get_param("starting_config")
joint_limits = get_param("joint_limits")
println(joint_limits)

function limit(value,lower,upper)
    if upper >= value >= lower
        return value
    elseif upper < value
        return upper
    elseif lower > value
        return lower
    end
end

goal = DebugGoals()
pose = Pose()
pose.position.x = 0
pose.position.y = 0
pose.position.z = 0
pose.orientation.w = 1
pose.orientation.x = 0
pose.orientation.y = 0
pose.orientation.z = 0
for i = 1:num_chains
    push!(goal.ee_poses, pose)
end
for i = 1:length(starting_config)
    push!(goal.dc_values,starting_config[i])
end
goal.eval_type = "null"
goal.bias = Point(1,1,1)
goal.header.stamp = get_rostime()

for i=1:length(objectives)
    push!(goal.lively_weights,objectives[i]["weight"])
    if occursin("noise",objectives[i]["type"])
        push!(goal.normal_weights,0.0)
    else
        push!(goal.normal_weights,objectives[i]["weight"])
    end
end

function solve_with_goal(data, livelyIK, relaxedIK)
    pose_goals = data.ee_poses
    dc_goals = data.dc_values

    pos_goals = []
    quat_goals = []

    for i = 1:num_chains
        p = pose_goals[i]

        pos_x = p.position.x
        pos_y = p.position.y
        pos_z = p.position.z

        quat_w = p.orientation.w
        quat_x = p.orientation.x
        quat_y = p.orientation.y
        quat_z = p.orientation.z

        push!(pos_goals, [pos_x, pos_y, pos_z])
        push!(quat_goals, Quat(quat_w, quat_x, quat_y, quat_z))
    end

    t = to_sec(data.header.stamp)

    bias = [data.bias.x,data.bias.y,data.bias.z]

    rxopt = solve(relaxedIK, pos_goals, quat_goals, dc_goals, t, [0,0,0], data.normal_weights)
    lxopt = solve(livelyIK,  pos_goals, quat_goals, dc_goals, t, bias,    data.lively_weights)

    ## Remove for simple execution
    # Try to determine an "Ideal" Noise, and also the Direct Perlin Noise
    weights = [[0] for i=1:length(objectives)]
    for i = 1:length(objectives)
        if objectives[i]["type"] == "dc_noise"
            # Assume equal weighting
            push!(weights[i],1)
        elseif objectives[i]["type"] == "positional_noise"
            push!(weights[i],data.lively_weights[i])
        end
    end

    relative_weights = zeros(length(objectives))

    for i = 1:length(objectives)
        if sum(weights[i]) == 0
            relative_weights[i] = 0
        elseif objectives[i]["type"] == "dc_noise"
            relative_weights[i] = 1 / sum(weights[i])
        elseif objectives[i]["type"] == "positional_noise"
            relative_weights[i] = data.lively_weights[i] / sum(weights[i])
        end
    end

    # Calculate Direct-Perlin (outside optimization)
    dc_noise = zeros(length(starting_config))
    ee_noise = [[0.0,0.0,0.0] for i=1:length(starting_config)]

    for i = 1:length(objectives)
        if objectives[i]["type"] == "dc_noise"
            dc_noise[objectives[i]["index"]] = dc_noise[objectives[i]["index"]] + livelyIK.relaxedIK_vars.noise.generators[i].value * relative_weights[i]
        elseif objectives[i]["type"] == "positional_noise"
            ee_noise[objectives[i]["index"]] = ee_noise[objectives[i]["index"]] + livelyIK.relaxedIK_vars.noise.generators[i].value * relative_weights[i]
        end
    end

    ## Remove above for simple exection

    ideal_goals = []
    for i = 1:num_chains
        pos_goal = pos_goals[i] .+ ee_noise[i]
        pos_noise_goal = Point(pos_goal[1],pos_goal[2],pos_goal[3])
        # Ignoring orientation for now
        pose = Pose()
        pose.position.x = pos_goal[1]
        pose.position.y = pos_goal[2]
        pose.position.z = pos_goal[3]
        pose.orientation.w = 1
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        push!(ideal_goals,pose)
    end

    dpa = DebugPoseAngles()
    for i = 1:length(rxopt)
        push!(dpa.angles_relaxed, rxopt[i])
        push!(dpa.angles_perlin, limit(rxopt[i]+dc_noise[i],joint_limits[i,1],joint_limits[i,2]))
    end
    for i = 1:length(lxopt)
        push!(dpa.angles_lively, lxopt[i])
    end

    dpa.collision_relaxed = in_collision_groundtruth(relaxedIK,rxopt)
    dpa.collision_lively = in_collision_groundtruth(relaxedIK,lxopt)
    dpa.collision_perlin = in_collision_groundtruth(relaxedIK,dpa.angles_perlin)

    dpa.ideal_noise = ideal_goals
    dpa.dc_values = data.dc_values
    dpa.ee_poses  = data.ee_poses
    dpa.header.seq = data.header.seq
    dpa.header.stamp = get_rostime()
    dpa.header.frame_id = data.header.frame_id
    dpa.eval_type = data.eval_type
    dpa.i = data.i
    return dpa

end

function handle_goals(data)
    global relaxedIK
    global livelyIK
    global angles_pub
    sol = solve_with_goal(data, livelyIK, relaxedIK)
    publish(angles_pub,sol)
end


loop_rate = Rate(60)
quit = false
loginfo("Finalizing JIT Compilation")
set_goal_info(goal)
dpa = solve_with_goal(goal_info, livelyIK, relaxedIK)
loginfo("Finished")
Subscriber{DebugGoals}("/relaxed_ik/debug_goals", handle_goals)
set_param("ready",true)

spin()
