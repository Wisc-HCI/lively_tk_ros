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
global lik_info_data = YAML.load(info_string)
global rik_info_data = YAML.load(info_string)
for i=1:length(rik_info_data["objectives"])
    objective = rik_info_data["objectives"][i]
    if objective["type"] == "positional_noise" || objective["type"] == "rotational_noise" || objective["type"] == "dc_noise"
        rik_info_data["objectives"][i]["weight"] = 0
    end
end
rik_info_data["fixed_frame_noise_scale"] = 0

function strip_noise(objectives,weights)
    for i=1:length(objectives)
        if objectives[i]["type"] == "positional_noise" || objectives[i]["type"] == "rotational_noise" || objectives[i]["type"] == "dc_noise"
            weights[i] = 0
        end
    end
    return weights
end

# Create JS Publisher
global solutions_pub = node.create_publisher(wisc_msgs.EvalResult,"/eval_results",5)

# LivelyIK Setup
global lik = LivelyIK.get_standard(lik_info_data)
global rik = LivelyIK.get_standard(rik_info_data)

global goals = LivelyIK.Goals(lik,lik_info_data)

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
    rik_weights = strip_noise(rik_info_data["objectives"],goals.weights)
    lik_sol = LivelyIK.solve(lik, goals.positions, goals.quats, goals.dc, time, goals.bias, goals.weights)
    rik_sol = LivelyIK.solve(rik, goals.positions, goals.quats, goals.dc, time, [0.0,0.0,0.0], rik_weights)
    # println("SOL: $sol")
    msg = wisc_msgs.EvalResult(metadata=goals.metadata,lively_joints=lik_sol,relaxed_joints=rik_sol)
    msg.header.stamp = node.get_clock().now().to_msg()
    solutions_pub.publish(msg)
end
