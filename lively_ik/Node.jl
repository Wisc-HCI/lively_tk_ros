#!/bin/bash
# -*- mode: julia -*-
#=
exec julia --color=yes --startup-file=no "${BASH_SOURCE[0]}" "$@"
=#

using YAML
using LivelyIK
using PyCall
using Rotations

rclpy = pyimport("rclpy")
rclpy_node = pyimport("rclpy.node")
rclpy_time = pyimport("rclpy.time")
wisc_msgs = pyimport("wisc_msgs.msg")
std_msgs = pyimport("std_msgs.msg")
sensor_msgs = pyimport("sensor_msgs.msg")
rcl_srv = pyimport("rcl_interfaces.srv")

# ROS and LivelyIK Setup
rclpy.init()
global node = rclpy_node.Node("ik_node")

# Get parameters (info data)
param_client = node.create_client(rcl_srv.GetParameters, "/global_params/get_parameters")
request = rcl_srv.GetParameters.Request()
request.names = ["info","output_topic"]
future = param_client.call_async(request)
rclpy.spin_until_future_complete(node, future)
response = future.result()
info_string = response.values[1].string_value
output_topic = response.values[2].string_value
global info_data = YAML.load(info_string)

# function msg_to_args(goal_msg)
#     goal_positions = []
#     goal_quats = []
#
#     # Create POS/QUAT Goals from EE Poses
#     for i = 1:length(goal_msg.ee_poses)
#         p = goal_msg.ee_poses[i]
#
#         pos_x = p.position.x
#         pos_y = p.position.y
#         pos_z = p.position.z
#
#         quat_w = p.orientation.w
#         quat_x = p.orientation.x
#         quat_y = p.orientation.y
#         quat_z = p.orientation.z
#
#         push!(goal_positions, [pos_x, pos_y, pos_z])
#         push!(goal_quats, Quat(quat_w, quat_x, quat_y, quat_z))
#     end
#
#     # Create DC Goals from DC Values
#     dc_goals = goal_msg.dc_values
#
#     # Get Time from Header
#     time = rclpy_time.Time.from_msg(goal_msg.header.stamp).nanoseconds * 10^-9
#
#     # Extract Bias
#     bias = [goal_msg.bias.x,goal_msg.bias.y,goal_msg.bias.z]
#
#     # Extract Weights
#     weights = goal_msg.objective_weights
#
#     return goal_positions, goal_quats, dc_goals, time, bias, weights
# end
#
# function get_initial_goals(lively_ik,info_data)
#     num_chains = lively_ik.relaxedIK_vars.robot.num_chains
#
#     goal_positions = []
#     goal_quats = []
#
#     # Create POS/QUAT Goals from EE Poses
#     for i = 1:length(num_chains)
#         push!(goal_positions, [0.0, 0.0, 0.0])
#         push!(goal_quats, Quat(1.0, 0.0, 0.0, 0.0))
#     end
#
#     # Create DC Goals from DC Values
#     dc_goals = info_data["starting_config"]
#
#     # Get Time from Header
#     time = 0
#
#     # Extract Bias
#     bias = [1,1,1]
#
#     # Extract Weights
#     weights = lively_ik.relaxedIK_vars.vars.weight_priors
#
#     return goal_positions, goal_quats, dc_goals, time, bias, weights
# end

# Create JS Publisher
global solutions_pub = node.create_publisher(sensor_msgs.JointState,output_topic,5)

# function publish()
#     global xopt
#     global solutions_pub
#     global node
#     # println("In Publish: $xopt")
#
# end
#
# timer = node.create_timer(0.05,publish)

# LivelyIK Setup
global lik = LivelyIK.get_standard(info_data,node)

global goals = LivelyIK.Goals(lik,info_data)

function goal_cb(msg)
    # Handle the goal message
    global goals
    time = rclpy_time.Time.from_msg(msg.header.stamp).nanoseconds * 10^-9
    LivelyIK.update!(goals,msg,time)
end

goal_sub = node.create_subscription(wisc_msgs.LivelyGoals,"/robot_goals",goal_cb,5)

# Process Until Interrupted
println("\033[92mRunning LivelyIK Node\033[0m")
while rclpy.ok()
    global node
    global goals
    rclpy.spin_once(node)
    time = node.get_clock().now().nanoseconds * 10^-9
    sol = LivelyIK.solve(lik, goals.positions, goals.quats, goals.dc, time, goals.bias, goals.weights)
    msg = sensor_msgs.JointState(name=info_data["joint_ordering"],position=sol)
    msg.header.stamp = node.get_clock().now().to_msg()
    solutions_pub.publish(msg)
end
