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
@rosimport wisc_msgs.msg: DCPoseGoals, EEPoseGoals, JointAngles, DebugPoseAngles
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

function eePoseGoals_cb(data::EEPoseGoals)
    global eepg
    #loginfo("$data")
    eepg = data
end
function dcPoseGoals_cb(data::DCPoseGoals)
    global dcpg
    dcpg = data
    #values = dcpg.dc_values
    #println("Values Update: $values")
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


Subscriber{EEPoseGoals}("/relaxed_ik/ee_pose_goals", eePoseGoals_cb)
Subscriber{DCPoseGoals}("/relaxed_ik/dc_pose_goals", dcPoseGoals_cb)
Subscriber{BoolMsg}("/relaxed_ik/quit", quit_cb, queue_size=1)
Subscriber{BoolMsg}("/relaxed_ik/reset", reset_cb)
angles_pub = Publisher("/relaxed_ik/debug_pose_angles", DebugPoseAngles, queue_size = 3)

sleep(0.5)

eepg = EEPoseGoals()
pose = Pose()
pose.position.x = -0.12590331808269600
pose.position.y = 0.23734846974527900
pose.position.z = 0.3734423326681300
pose.orientation.w = 0.5046115849968640
pose.orientation.x = -0.4993768344058750
pose.orientation.y = 0.5065220290165270
pose.orientation.z = 0.48931110723822800
for i = 1:num_chains
    push!(eepg.ee_poses, pose)
end
dcpg = DCPoseGoals()
for i = 1:num_dc
    push!(dcpg.dc_values,0.5)
end
empty_eepg = eepg
empty_dcpg = dcpg

loop_rate = Rate(40)
quit = false
loginfo("starting")
while !is_shutdown()
    global quit
    global reset_solver
    global eepg
    global dcpg
    global relaxedIK
    global livelyIK
    global xopt

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
        eepg = empty_eepg
        dcpg = empty_dcpg
    end

    pose_goals = eepg.ee_poses
    dc_goals = dcpg.dc_values
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
    #xopt = solve_precise(relaxedIK, pos_goals, quat_goals, dc_goals, time, priority)[1]
    rxopt = solve(relaxedIK, pos_goals, quat_goals, dc_goals, time, 1, [0,0,0])
    lxopt = solve(livelyIK, pos_goals, quat_goals, dc_goals, time, 0, [1,1,1])
    # loginfo("xopt: $xopt")

    dpa = DebugPoseAngles()
    for i = 1:length(rxopt)
        push!(dpa.angles_relaxed.data, rxopt[i])
    end
    for i = 1:length(lxopt)
        push!(dpa.angles_lively.data, lxopt[i])
    end
    dpa.dc_values = dcpg
    dpa.ee_poses  = eepg
    dpa.header.seq = eepg.header.seq
    dpa.header.stamp = get_rostime()#eepg.header.stamp
    dpa.header.frame_id = eepg.header.frame_id
    # println(ja.angles)

    publish(angles_pub, dpa)

    rossleep(loop_rate)
end
