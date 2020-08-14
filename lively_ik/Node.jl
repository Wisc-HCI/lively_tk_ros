#!/bin/bash
# -*- mode: julia -*-
#=
exec julia --color=yes --startup-file=no "${BASH_SOURCE[0]}" "$@"
=#

using YAML
# using ArgParse
using LivelyIK
using PyCall
using Rotations

rclpy = pyimport("rclpy")
rclpy_node = pyimport("rclpy.node")
rclpy_time = pyimport("rclpy.time")
wisc_msgs = pyimport("wisc_msgs.msg")

# s = ArgParseSettings()
# @add_arg_table! s begin
#     "--info_file", "-i"
#         arg_type = String
#         help = "Full path to the info file"
#         required = true
# end

function msg_to_args(goal_msg)
    goal_positions = []
    goal_quats = []

    # Create POS/QUAT Goals from EE Poses
    for i = 1:length(goal_msg.ee_poses)
        p = goal_msg.ee_poses[i]

        pos_x = p.position.x
        pos_y = p.position.y
        pos_z = p.position.z

        quat_w = p.orientation.w
        quat_x = p.orientation.x
        quat_y = p.orientation.y
        quat_z = p.orientation.z

        push!(goal_positions, [pos_x, pos_y, pos_z])
        push!(goal_quats, Quat(quat_w, quat_x, quat_y, quat_z))
    end

    # Create DC Goals from DC Values
    dc_goals = goal_msg.dc_values

    # Get Time from Header
    time = rclpy_time.Time.from_msg(goal_msg.header.stamp).nanoseconds * 10^-9

    # Extract Bias
    bias = [goal_msg.bias.x,goal_msg.bias.y,goal_msg.bias.z]

    # Extract Weights
    weights = goal_msg.objective_weights

    return goal_positions, goal_quats, dc_goals, time, bias, weights
end

function get_initial_goals(lively_ik,info)
    num_chains = lively_ik.relaxedIK_vars.robot.num_chains

    goal_positions = []
    goal_quats = []

    # Create POS/QUAT Goals from EE Poses
    for i = 1:length(num_chains)
        push!(goal_positions, [0, 0, 0])
        push!(goal_quats, Quat(1, 0, 0, 0))
    end

    # Create DC Goals from DC Values
    dc_goals = info["starting_config"]

    # Get Time from Header
    time = 0

    # Extract Bias
    bias = [1,1,1]

    # Extract Weights
    weights = lively_ik.relaxedIK_vars.vars.weight_priors

    return goal_positions, goal_quats, dc_goals, time, bias, weights
end

function xopt_to_msg(time, xopt)
    solution_msg = wisc_msgs.JointAngles()
    solution_msg.header.stamp = time
    for i = 1:length(xopt)
        push!(solution_msg.angles,xopt[i])
    end
    return solution_msg
end

# parsed_args = parse_args(ARGS, s)

# info_path = parsed_args["info_file"]
info_data = YAML.load(open("/Users/schoen/ROS/ros2_lik/install/lively_ik/share/lively_ik/config/info_files/info_ur3e.yaml"))

# ROS and LivelyIK Setup
rclpy.init()
node = rclpy_node.Node("lively_ik_node")

#info_data2 = node.get_parameter("/lively_ik/info")
#println(info_data2)

lik = LivelyIK.get_standard(info_data,rcl_node=node)


solutions_pub = node.create_publisher(wisc_msgs.JointAngles,"/relaxed_ik/joint_angles",5)

function goal_cb(goal_msg)
    # Handle the goal message and publish the solution from lively_ik
    goal_positions, goal_quats, dc_goals, time, bias, weights = msg_to_args(goal_msg)
    xopt = solve(lik, goal_positions, goal_quats, dc_goals, time, bias, weights)
    solutions_pub.publish(xopt_to_msg(time,xopt))
end

goal_sub = node.create_subscription(wisc_msgs.LivelyGoals,"/relaxed_ik/goals",goal_cb,5)

# Solve once
println("Finishing Compilation")
goal_positions, goal_quats, dc_goals, time, bias, weights = get_initial_goals(lik,info_data)
sol = LivelyIK.solve(lik, goal_positions, goal_quats, dc_goals, time, bias, weights)
println(sol)

# Process Until Interrupted
println("Running LivelyIK Node")
rclpy.spin(node)
