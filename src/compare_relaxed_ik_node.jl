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
@rosimport wisc_msgs.msg: DCPoseGoals, EEPoseGoals, JointAngles, DebugPoseAngles, DebugGoals
@rosimport std_msgs.msg: Float64MultiArray, Bool, Float64
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


init_node("lively_ik_node")

path_to_src = Base.source_dir()

loaded_robot_file = open(path_to_src * "/RelaxedIK/Config/loaded_robot")
loaded_robot = readline(loaded_robot_file)
close(loaded_robot_file)
relaxedIK = get_standard(path_to_src, loaded_robot)
livelyIK = get_standard(path_to_src, loaded_robot)
num_chains = relaxedIK.relaxedIK_vars.robot.num_chains
num_dc = relaxedIK.relaxedIK_vars.noise.num_dc

goal_info = DebugGoals()
pose = Pose()
pose.position.x = -0.12590331808269600
pose.position.y = 0.23734846974527900
pose.position.z = 0.3734423326681300
pose.orientation.w = 0.5046115849968640
pose.orientation.x = -0.4993768344058750
pose.orientation.y = 0.5065220290165270
pose.orientation.z = 0.48931110723822800
for i = 1:num_chains
    push!(goal_info.ee_poses, pose)
end
for i = 1:num_dc
    push!(goal_info.dc_values,0.5)
end
goal_info.priority = 0
goal_info.eval_type = "null"
goal_info.bias = Point(1,1,1)
empty_goal_info = goal_info

angles_pub = Publisher("/relaxed_ik/debug_pose_angles", DebugPoseAngles, queue_size = 3)

function goals_cb(data::DebugGoals)
    global quit
    global reset_solver
    global relaxedIK
    global livelyIK
    global xopt
    global goal_info

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
    # println(dc_goals)

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

    #loginfo("pos_goals: $pos_goals")
    #loginfo("quat_goals: $quat_goals")
    #loginfo("dc_goals: $dc_goals")

    time = to_sec(get_rostime())/4
    # priority = get_param("/lively_ik/priority")
    # bias = get_param("/lively_ik/bias")
    priority = goal_info.priority
    bias = [goal_info.bias.x,goal_info.bias.y,goal_info.bias.z]
    #xopt = solve_precise(relaxedIK, pos_goals, quat_goals, dc_goals, time, priority)[1]
    rxopt = solve(relaxedIK, pos_goals, quat_goals, dc_goals, time, 1, [0,0,0],discontinuous=!goal_info.continuous)
    lxopt = solve(livelyIK, pos_goals, quat_goals, dc_goals, time, priority, bias, discontinuous=!goal_info.continuous)
    # loginfo("xopt: $xopt")

    dpa = DebugPoseAngles()
    for i = 1:length(rxopt)
        push!(dpa.angles_relaxed, rxopt[i])
    end
    for i = 1:length(lxopt)
        push!(dpa.angles_lively, lxopt[i])
    end
    dpa.dc_values = goal_info.dc_values
    dpa.ee_poses  = goal_info.ee_poses
    dpa.header.seq = goal_info.header.seq
    dpa.header.stamp = get_rostime()#goal_info.header.stamp
    dpa.header.frame_id = goal_info.header.frame_id
    dpa.eval_type = goal_info.eval_type
    # println(ja.angles)

    publish(angles_pub, dpa)
    println("Published update")
end


println("loaded robot: $loaded_robot")

# Test the callback
goals_cb(goal_info)

Subscriber{DebugGoals}("/relaxed_ik/debug_goals", goals_cb)
Subscriber{BoolMsg}("/relaxed_ik/quit", quit_cb, queue_size=1)
Subscriber{BoolMsg}("/relaxed_ik/reset", reset_cb)


println("Lively_IK Ready!")

for i = 1:400
    goals_cb(goal_info)
end

spin()
