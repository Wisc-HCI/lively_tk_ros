
# objectives should be written with respect to x and vars.  Closure will be made later automatically.

using LinearAlgebra
using StaticArrays
using Rotations
include("../utils/transformations.jl")
include("../utils/joint_utils.jl")
include("../utils/geometry_utils.jl")
include("../utils/nn_utils.jl")

function groove_loss(x_val, t, d, c, f, g)
    return (-2.718281828459^((-(x_val - t)^d) / (2.0 * c^2)) ) + f * (x_val - t)^g
end

function groove_loss_derivative(x_val, t, d, c, f, g)
    return -2.718281828459^((-(x_val - t)^d) / (2.0 * c^2)) * ( (-d*(x_val-t) ) / (2. * c^2) ) + g*f*(x_val-t)
end

function dc_noise_obj(x, vars, objidx, jointidx)
    # Calculate the delta between goal and state
    goal = vars.joint_goal[jointidx]+vars.noise.generators[objidx].value
    x_val = abs(x[jointidx]-goal)

    # return groove_loss(  x_val, 0.0, 2.0, 2.3, 0.003, 2.0 )
    return groove_loss(  x_val, 0.0, 2, 0.3295051144911304, 0.1, 2)
end


function positional_noise_obj(x, vars, objidx, eeidx)
    #println("POS NOISE IDX: $objidx $eeidx")
    vars.robot.arms[eeidx].getFrames(x[vars.robot.subchain_indices[eeidx]])
    #println("POS 1")
    goal = vars.goal_positions[eeidx] + vars.noise.generators[objidx].value + vars.noise.base.value
    #println("POS 2")
    x_val = norm(vars.robot.arms[eeidx].out_pts[end] - goal)
    #println("POS 3")
    return groove_loss(x_val, 0., 2, .1, 10., 2)
end

function rotational_noise_obj(x, vars, objidx, eeidx)
    #println("ROT NOISE IDX: $objidx $eeidx")
    vars.robot.arms[eeidx].getFrames(x[vars.robot.subchain_indices[eeidx]])
    #println("ROT 1")
    eeMat = vars.robot.arms[eeidx].out_frames[end]
    #println("ROT 2")
    orilog = quaternion_log(vars.goal_quats[eeidx])
    #println("ROT 3")
    goal_ori = orilog + vars.noise.generators[objidx].value
    #println("ROT 4")
    goal_quat = quaternion_exp(goal_ori)
    #println("ROT 5")
    ee_quat = Quat(eeMat)
    #println("ROT 6")
    ee_quat2 = Quat(-ee_quat.w, -ee_quat.x, -ee_quat.y, -ee_quat.z)
    #println("ROT 7")
    disp = norm(quaternion_disp(goal_quat, ee_quat))
    disp2 = norm(quaternion_disp(goal_quat, ee_quat2))

    x_val = min(disp, disp2)

    # return groove_loss(x_val, 0.,2.,.1,10.,2.)
    return groove_loss(x_val, 0., 2, .1, 10., 2)
end

function joint_match_obj(x, vars, idx1, idx2)
    x_val = abs(x[idx1]-x[idx2])
    return groove_loss(x_val, 0., 2, .1, 10., 2)
end

function orientation_match_obj(x, vars, idx1, idx2)

    vars.robot.arms[idx1].getFrames(x[vars.robot.subchain_indices[idx1]])
    vars.robot.arms[idx2].getFrames(x[vars.robot.subchain_indices[idx2]])
    eeMat1 = vars.robot.arms[idx1].out_frames[end]
    eeMat2 = vars.robot.arms[idx2].out_frames[end]
    ee_quat1 = Quat(eeMat1)
    ee_quat2 = Quat(eeMat2)
    ee_quat3 = Quat(-ee_quat2.w,-ee_quat2.x,-ee_quat2.y,-ee_quat2.z)
    disp1 = norm(quaternion_disp(ee_quat1, ee_quat2))
    disp2 = norm(quaternion_disp(ee_quat1, ee_quat3))
    x_val = min(disp1, disp2)

    return groove_loss(x_val, 0., 2, .1, 10., 2)
end

function x_match_obj(x, vars, idx1, idx2)

    vars.robot.arms[idx1].getFrames(x[vars.robot.subchain_indices[idx1]])
    vars.robot.arms[idx2].getFrames(x[vars.robot.subchain_indices[idx2]])
    x_1 = vars.robot.arms[idx1].out_pts[end][1]
    x_2 = vars.robot.arms[idx2].out_pts[end][1]
    x_val = abs(x_1 - x_2)

    return groove_loss(x_val, 0., 2, .1, 10., 2)
end

function y_match_obj(x, vars, idx1, idx2)

    vars.robot.arms[idx1].getFrames(x[vars.robot.subchain_indices[idx1]])
    vars.robot.arms[idx2].getFrames(x[vars.robot.subchain_indices[idx2]])
    y_1 = vars.robot.arms[idx1].out_pts[end][2]
    y_2 = vars.robot.arms[idx2].out_pts[end][2]
    x_val = abs(y_1 - y_2)

    return groove_loss(x_val, 0., 2, .1, 10., 2)
end

function z_match_obj(x, vars, idx1, idx2)

    vars.robot.arms[idx1].getFrames(x[vars.robot.subchain_indices[idx1]])
    vars.robot.arms[idx2].getFrames(x[vars.robot.subchain_indices[idx2]])
    z_1 = vars.robot.arms[idx1].out_pts[end][3]
    z_2 = vars.robot.arms[idx2].out_pts[end][3]
    x_val = abs(z_1 - z_2)

    return groove_loss(x_val, 0., 2, .1, 10., 2)
end