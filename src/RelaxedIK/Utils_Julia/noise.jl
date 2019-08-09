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
    time
    arm_noise
    dc_noise
    base_noise
    arm_scale
    dc_scale
    base_scale
    arm_seed
    dc_seed
    base_seed
    global_seed
    dc_names
end

function NoiseGenerator(arm_scale, base_scale, dc_names, dc_scale)
    num_chains = length(arm_scale)
    num_dc = length(dc_scale)

    arm_noise = []
    dc_noise  = []
    arm_seed  = []
    dc_seed   = []

    for i=1:num_chains
        push!(arm_noise, posenoise(zeros(3),zeros(3)))
        push!(arm_seed,  posenoise(10*rand(3),10*rand(3)))
    end
    for i=1:num_dc
        push!(dc_noise, 0)
        push!(dc_seed,  10*rand())
    end

    base_noise = posenoise(zeros(3),zeros(3))
    base_seed  = posenoise(10*rand(3),10*rand(3))
    global_seed = rand()*10

    # Remove in final
    n = NoiseGenerator(num_chains, num_dc, time, arm_noise, dc_noise, base_noise, arm_scale, dc_scale, base_scale, arm_seed, dc_seed, base_seed, global_seed, dc_names)

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

    # Handle End Effector Noise
    for i=1:noisegen.num_chains
        if noisegen.arm_scale[i] > 0.0
            noisegen.arm_noise[i].position = noise3D(noisegen.time,noisegen.arm_seed[i].position) * noisegen.arm_scale[i] * scale * limit(time,noisegen.global_seed)
            noisegen.arm_noise[i].rotation = noise3D(noisegen.time,noisegen.arm_seed[i].rotation) * noisegen.arm_scale[i] * 0.3 * scale
        end
    end
    # Handle Direct Control
    for i=1:noisegen.num_dc
        if noisegen.dc_scale[i] > 0.0
            noisegen.dc_noise[i] = noise(noisegen.time,noisegen.dc_seed[i]) * noisegen.dc_scale[i]
        end
    end
    # Handle fixed frame noise
    if noisegen.base_scale > 0.0
        noisegen.base_noise.position = noise3D(noisegen.time,noisegen.base_seed.position) * noisegen.base_scale * scale * limit(time,noisegen.global_seed)
        noisegen.base_noise.rotation = noise3D(noisegen.time,noisegen.base_seed.rotation) * noisegen.base_scale * 0.3 * scale
    end

end
