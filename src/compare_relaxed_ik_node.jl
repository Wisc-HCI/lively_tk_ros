#!/usr/bin/env julia
include("RelaxedIK/relaxedIK.jl")
include("RelaxedIK/GROOVE_RelaxedIK_Julia/relaxedIK_vars.jl")
include("RelaxedIK/Utils_Julia/solver_output.jl")
using YAML
using RobotOS
using Rotations
# using BenchmarkTools
using ForwardDiff
# using Knet
# using Dates
@rosimport wisc_msgs.msg: DCPoseGoals, EEPoseGoals, JointAngles, DebugGoals, DebugPoseAngles
@rosimport std_msgs.msg: Float64MultiArray, Bool, Float32
@rosimport geometry_msgs.msg: Point, Quaternion, Pose

rostypegen()
using .wisc_msgs.msg
using .std_msgs.msg
using .geometry_msgs.msg

quit = false
function quit_cb(data::BoolMsg)
    global quit
    quit = data.data
end

reset_solver = false
function reset_cb(data::BoolMsg)
    global reset_solver
    reset_solver = data.data
end

eepg = Nothing
dcpg = Nothing
bias = [1.0,1.0,1.0]

function goals_cb(data::DebugGoals)
    global goal_info
    #loginfo("$data")
    goal_info = data
end

init_node("lively_ik_node")

path_to_src = Base.source_dir()

loaded_robot_file = open(path_to_src * "/RelaxedIK/Config/loaded_robot")
loaded_robot = readline(loaded_robot_file)
close(loaded_robot_file)
relaxedIK = get_standard(path_to_src, loaded_robot)
livelyIK = get_standard(path_to_src, loaded_robot)
num_chains = relaxedIK.relaxedIK_vars.robot.num_chains
num_dc = relaxedIK.relaxedIK_vars.noise.num_dc

println("loaded robot: $loaded_robot")


Subscriber{DebugGoals}("/relaxed_ik/debug_goals", goals_cb)
Subscriber{BoolMsg}("/relaxed_ik/quit", quit_cb, queue_size=1)
Subscriber{BoolMsg}("/relaxed_ik/reset", reset_cb)
angles_pub = Publisher("/relaxed_ik/debug_pose_angles", DebugPoseAngles, queue_size = 3)

sleep(0.5)

goal_info = DebugGoals()
pose = Pose()
pose.position.x = 0
pose.position.y = 0
pose.position.z = 0
pose.orientation.w = 1
pose.orientation.x = 0
pose.orientation.y = 0
pose.orientation.z = 0
for i = 1:num_chains
    push!(goal_info.ee_poses, pose)
end
for i = 1:num_dc
    push!(goal_info.dc_values,0.5)
end
goal_info.eval_type = "null"
goal_info.bias = Point(1,1,1)
goal_info.normal_weights = [50.0, 20.0, 0.00, 0.00, 5.0, 3.0, 0.2, 1.0, 2.0]
goal_info.lively_weights = [25.0, 10.0, 25.0, 10.0, 5.0, 3.0, 0.2, 1.0, 2.0]
empty_goal_info = goal_info

loop_rate = Rate(60)
quit = false
loginfo("starting")
started = false

while !is_shutdown()
    global quit
    global reset_solver
    global relaxedIK
    global livelyIK
    global xopt
    global goal_info
    global empty_goal_info
    global started

    if quit == true
        println("quitting")
        quit = false
        return
    end

    if reset_solver == true
        println("resetting")
        reset_solver = false
        relaxedIK = get_standard(path_to_src, loaded_robot)
        livelyIK = get_standard(path_to_src, loaded_robot)
        goal_info = empty_goal_info
    end

    pose_goals = goal_info.ee_poses
    dc_goals = goal_info.dc_values

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

    time = to_sec(goal_info.header.stamp)

    bias = [goal_info.bias.x,goal_info.bias.y,goal_info.bias.z]
    # normal_weights = [50.0, 20.0, 0.00, 0.00, 5.0, 3.0, 0.2, 1.0, 2.0]
    # lively_weights = [25.0, 10.0, 25.0, 10.0, 5.0, 3.0, 0.2, 1.0, 2.0]

    # rxopt, rtry_idx, rvalid_sol, rpos_error, rrot_error = solve_precise(relaxedIK, pos_goals, quat_goals, dc_goals, time, [0,0,0], normal_weights)
    # lxopt, ltry_idx, lvalid_sol, lpos_error, lrot_error = solve_precise(livelyIK,  pos_goals, quat_goals, dc_goals, time, bias,    lively_weights)
    rxopt = solve(relaxedIK, pos_goals, quat_goals, dc_goals, time, [0,0,0], goal_info.normal_weights)
    lxopt = solve(livelyIK,  pos_goals, quat_goals, dc_goals, time, bias,    goal_info.lively_weights)
    ideal_noise = livelyIK.relaxedIK_vars.noise.arm_noise

    ideal_goals = []
    for i = 1:num_chains
        pos_goal = pos_goals[i] .+ ideal_noise[i].position
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
    end
    for i = 1:length(lxopt)
        push!(dpa.angles_lively, lxopt[i])
    end
    dpa.ideal_noise = ideal_goals
    dpa.dc_values = goal_info.dc_values
    dpa.ee_poses  = goal_info.ee_poses
    dpa.header.seq = goal_info.header.seq
    dpa.header.stamp = get_rostime()#goal_info.header.stamp
    dpa.header.frame_id = goal_info.header.frame_id
    dpa.eval_type = goal_info.eval_type
    dpa.i = goal_info.i
    # println(ja.angles)

    if started
        publish(angles_pub, dpa)
    else
        started = true
        println("Lively_IK Ready!")
        set_param("ready",true)
    end

    rossleep(loop_rate)
end
