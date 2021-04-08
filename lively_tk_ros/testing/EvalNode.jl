#!/bin/bash
# -*- mode: julia -*-
#=
exec julia --color=yes --startup-file=no "${BASH_SOURCE[0]}" "$@"
=#

using YAML
using JSON
using LivelyIK
using Rotations

global lik_info_data = YAML.load_file(ARGS[1])
global rik_info_data = YAML.load_file(ARGS[1])
global goals = JSON.parsefile(ARGS[2])
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

# LivelyIK Setup
global lik = LivelyIK.get_standard(lik_info_data)
global rik = LivelyIK.get_standard(rik_info_data)

global solutions = []
println("\033[92mRunning LivelyIK Node\033[0m")
for i=1:length(goals)
    global solutions
    goal=goals[i]
    # println(goal)
    time = goal["time"]
    update = goal["update"]
    lik_weights = map(w->w,update["weight"])
    rik_weights = map(w->w,update["weight"])
    rik_weights = strip_noise(rik_info_data["objectives"],update["weight"])
    positions = map(p->[p["position"]["x"],p["position"]["y"],p["position"]["z"]],update["pose"])
    quats = map(p->Rotations.Quat(p["orientation"]["w"],p["orientation"]["x"],p["orientation"]["y"],p["orientation"]["z"]),update["pose"])
    dc = update["dc"]
    lik_bias = update["bias"]
    rik_bias = [0.0,0.0,0.0]

    lik_sol = LivelyIK.solve(lik, positions, quats, dc, time, lik_bias, lik_weights)
    rik_sol = LivelyIK.solve(rik, positions, quats, dc, time, rik_bias, rik_weights)

    diff = []
    for v=1:length(lik_sol)
        push!(diff,lik_sol[v]-rik_sol[v])
    end
    # println("$time: $diff")
    if goal["metadata"] != "buffer"
        push!(solutions,Dict("time"=>time,"lik_sol"=>lik_sol,"rik_sol"=>rik_sol,"dc"=>dc))
    end
end
out_file = replace(ARGS[2],".json" => "_sol.json")

open(out_file, "w") do io
    JSON.print(io,solutions)
end;
