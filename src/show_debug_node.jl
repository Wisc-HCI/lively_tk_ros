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
@rosimport wisc_msgs.msg: DebugGoals, DebugPoseAngles
@rosimport geometry_msgs.msg: Point, Quaternion, Pose

rostypegen()
using .wisc_msgs.msg
using .std_msgs.msg
using .geometry_msgs.msg

global goal_info = nothing

function set_goal_info(data)
    global goal_info
    goal_info = data
    loginfo(goal_info.eval_type)
end

function log_goal_info(data)
    loginfo(data)
end

init_node("show_debug_node")

Subscriber{DebugGoals}("/relaxed_ik/debug_goals", set_goal_info)

loop_rate = Rate(60)

while !is_shutdown()
    global goal_info

    log_goal_info(goal_info)

    rossleep(loop_rate)
end
