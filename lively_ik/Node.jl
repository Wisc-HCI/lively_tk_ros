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

# Create JS Publisher
global solutions_pub = node.create_publisher(sensor_msgs.JointState,output_topic,5)

# LivelyIK Setup
global lik = LivelyIK.get_standard(info_data,node)

global goals = LivelyIK.Goals(lik,info_data)

function goal_cb(msg)
    # Handle the goal message
    global goals
    # println("MSG: $msg")
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
    #println("GOALS: $goals")
    time = node.get_clock().now().nanoseconds * 10^-9
    sol = LivelyIK.solve(lik, goals.positions, goals.quats, goals.dc, time, goals.bias, goals.weights)
    # println("SOL: $sol")
    msg = sensor_msgs.JointState(name=info_data["joint_ordering"],position=sol)
    msg.header.stamp = node.get_clock().now().to_msg()
    solutions_pub.publish(msg)
end
