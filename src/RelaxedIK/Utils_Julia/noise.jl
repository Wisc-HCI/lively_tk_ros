include("noise_utils.jl")

# Remove in final
using RobotOS
@rosimport geometry_msgs.msg: Point, Quaternion, Pose

mutable struct posenoise
    position
    rotation
end

mutable struct NoiseGenerator
    num_chains
    num_dc
    arms
    scale
    seeds
    time
    global_seed
    base_idx
    ndc
    xdc
    # Remove in final
    #pub
end

function NoiseGenerator(scale, base_link_noise, ndc, sdc)
    num_chains = length(scale)
    num_dc = length(ndc)
    # Add base link noise and direct control noise
    for i=1:num_dc
        push!(scale, sdc[i])
    end
    push!(scale,base_link_noise)
    arms  = []
    xdc   = []
    seeds = []

    for i=1:num_chains
        push!(arms, posenoise(zeros(3),zeros(3)))
        push!(seeds, posenoise(10*rand(3),10*rand(3)))
    end
    for i=1:num_dc
        push!(seeds, 10*rand())
        push!(xdc, 0)
    end
    global_seed = rand()*10

    # Remove in final
    n = NoiseGenerator(num_chains, num_dc, arms, scale, seeds, 0.0, global_seed, length(scale), ndc, xdc)

    update!(n, 0.0, 0.0)

    return n
end

function limit(time,seed)
    raw = noise(time,seed)
    limit = 1/(1+exp(5-10*abs(raw)))
    return limit
end

function update!(noisegen, time, priority)
    # Priority is a value 0-1.
    # Higher priority means less noise
    scale = 1-priority
    noisegen.time = time
    idx = 1
    # Handle End Effector Noise
    for i=1:noisegen.num_chains
        if noisegen.scale[idx] > 0.0
            noisegen.arms[idx].position = noise3D(noisegen.time,noisegen.seeds[idx].position) * noisegen.scale[idx] * scale * limit(time,noisegen.global_seed)
            noisegen.arms[idx].rotation = noise3D(noisegen.time,noisegen.seeds[idx].rotation) * noisegen.scale[idx] * 0.3 * scale
        end
        idx += 1
    end
    # Handle Direct Control
    for i=1:noisegen.num_dc
        if noisegen.scale[idx] > 0.0
            noisegen.xdc[idx] = noise(noisegen.time,noisegen.seeds[idx]) * noisegen.scale[idx]
        end
        idx += 1
    end
    # Handle fixed frame noise
    if noisegen.scale[idx] > 0.0
        noisegen.arms[idx].position = noise3D(noisegen.time,noisegen.seeds[idx].position) * noisegen.scale[idx] * scale * limit(time,noisegen.global_seed)
        noisegen.arms[idx].rotation = noise3D(noisegen.time,noisegen.seeds[idx].rotation) * noisegen.scale[idx] * 0.3 * scale
    end

end
