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
@rosimport lively_ik.msg : EEPoseGoals, DCPoseGoals, JointAngles
@rosimport std_msgs.msg: Float64MultiArray, Bool, Float32, Int8
@rosimport geometry_msgs.msg: Point, Quaternion, Pose

rostypegen()
using .lively_ik.msg
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
    eepg = data
end

function dcPoseGoals_cb(data::DCPoseGoals)
    global dcpg
    dcpg = data
end

init_node("relaxed_ik_node_jl")

path_to_src = Base.source_dir()
# println(path_to_src)
loaded_robot_file = open(path_to_src * "/RelaxedIK/Config/loaded_robot")
loaded_robot = readline(loaded_robot_file)
close(loaded_robot_file)
relaxedIK = get_standard(path_to_src, loaded_robot)
num_chains = relaxedIK.relaxedIK_vars.robot.num_chains
num_dc = relaxedIK.relaxedIK_vars.noise.num_dc

println("loaded robot: $loaded_robot")


Subscriber{EEPoseGoals}("/relaxed_ik/ee_pose_goals", eePoseGoals_cb)
Subscriber{DCPoseGoals}("/relaxed_ik/dc_pose_goals", dcPoseGoals_cb)
Subscriber{BoolMsg}("/relaxed_ik/quit", quit_cb, queue_size=1)
Subscriber{BoolMsg}("relaxed_ik/reset", reset_cb)
angles_pub = Publisher("/relaxed_ik/joint_angle_solutions", JointAngles, queue_size = 3)

sleep(0.5)

eepg = EEPoseGoals()
pose = Pose()
pose.position.x = 0.0
pose.position.y = 0.0
pose.position.z = 0.0
pose.orientation.w = 1.0
pose.orientation.x = 0.0
pose.orientation.y = 0.0
pose.orientation.z = 0.0
for i = 1:num_chains
    push!(eepg.ee_poses, pose)
end
dcpg = DCPoseGoals()
for i = 1:num_dc
    push!(dcpg.dc_values,Float32(0.5))
end
empty_eepg = eepg
empty_dcpg = dcpg

loop_rate = Rate(700)
quit = false
# println("Starting")
while !is_shutdown()
    global quit
    if quit == true
        println("quitting")
        quit = false
        return
    end

    global reset_solver
    global eepg
    global dcpg
    global relaxedIK
    global xopt

    if reset_solver == true
        println("resetting")
        reset_solver = false
        relaxedIK = get_standard(path_to_src, loaded_robot)
        eepg = empty_eepg
        dcpg = empty_dcpg
    end

    pose_goals = eepg.ee_poses
    dc_goals = dcpg.dc_values

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
    time = to_sec(get_rostime())/4
    xopt = solve(relaxedIK, pos_goals, quat_goals, dc_goals, time, 0)
    # println(relaxedIK.relaxedIK_vars.vars.objective_closures[end](xopt))
    ja = JointAngles()
    for i = 1:length(xopt)
        push!(ja.angles.data, xopt[i])
    end

    ja.header.seq = eepg.header.seq
    ja.header.stamp = eepg.header.stamp
    ja.header.frame_id = eepg.header.frame_id

    publish(angles_pub, ja)

    # println(in_collision(relaxedIK, xopt))
    rossleep(loop_rate)
end
