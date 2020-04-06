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
    arm_noise
    dc_noise
    base_noise
    position_scale
    rotation_scale
    dc_scale
    base_scale
    arm_seed
    dc_seed
    base_seed
    global_seed
end

function NoiseGenerator(position_scale, rotation_scale, dc_scale, base_scale)
    num_chains = length(position_scale)
    num_dc = length(dc_scale)

    arm_noise = []
    dc_noise  = zeros(num_dc)
    arm_seed  = []
    dc_seed   = []

    for i=1:num_chains
        push!(arm_noise, posenoise(zeros(3),zeros(3)))
        push!(arm_seed,  posenoise(1000*rand(3),1000*rand(3)))
    end
    for i=1:num_dc
        push!(dc_seed,  1000*rand())
    end

    base_noise = posenoise(zeros(3),zeros(3))
    base_seed  = posenoise(1000*rand(3),1000*rand(3))
    global_seed = rand()*1000

    n = NoiseGenerator(num_chains, num_dc, arm_noise, dc_noise, base_noise, position_scale, rotation_scale, dc_scale, base_scale, arm_seed, dc_seed, base_seed, global_seed)

    update!(n, 0.0, [1.0,1.0,1.0])

    return n
end

function limit(time,seed)
    raw = noise(time,seed)
    limit = 1/(1+exp(5-10*abs(raw)))
    return limit
end

function update!(noisegen, time, bias)
    # Handle End Effector Noise
    for i=1:noisegen.num_chains
        if noisegen.position_scale[i] > 0.0
            noisegen.arm_noise[i].position = noise3D(time,noisegen.arm_seed[i].position) .* noisegen.position_scale[i] .* bias# * limit(time,noisegen.global_seed)
        end
        if noisegen.rotation_scale[i] > 0.0
            noisegen.arm_noise[i].rotation = noise3D(time,noisegen.arm_seed[i].rotation) * noisegen.rotation_scale[i]
        end
    end
    # Handle Direct Control
    for i=1:noisegen.num_dc
        if noisegen.dc_scale[i] > 0.0
            noisegen.dc_noise[i] = noise(time,noisegen.dc_seed[i]) * noisegen.dc_scale[i]
        end
    end
    # Handle fixed frame noise
    if noisegen.base_scale > 0.0
        noisegen.base_noise.position = noise3D(time,noisegen.base_seed.position) * noisegen.base_scale .* bias
    end
end
