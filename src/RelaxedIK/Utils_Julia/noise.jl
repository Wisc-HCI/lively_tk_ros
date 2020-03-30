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
    dc_weight
end

function NoiseGenerator(arm_scale, base_scale, dc_scale, dc_weight)
    num_chains = length(arm_scale)
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
        if (dc_weight[i] > 0)
            push!(dc_seed,  1000*rand())
        else
            push!(dc_seed, 0)
        end
    end

    #dc_mask = zeros(length(dc_names))

    base_noise = posenoise(zeros(3),zeros(3))
    base_seed  = posenoise(1000*rand(3),1000*rand(3))
    global_seed = rand()*1000

    # Remove in final
    n = NoiseGenerator(num_chains, num_dc, time, arm_noise, dc_noise, base_noise, arm_scale, dc_scale, base_scale, arm_seed, dc_seed, base_seed, global_seed, dc_weight)

    update!(n, 0.0, 0.0, [1.0,1.0,1.0])

    return n
end

function limit(time,seed)
    raw = noise(time,seed)
    limit = 1/(1+exp(5-10*abs(raw)))
    return limit
end

function update!(noisegen, time, priority, bias)
    # Priority is a value 0-1.
    # Higher priority means less noise
    mag = 1-priority
    noisegen.time = time

    # Handle End Effector Noise
    for i=1:noisegen.num_chains
        if noisegen.arm_scale[i] > 0.0
            noisegen.arm_noise[i].position = noise3D(noisegen.time,noisegen.arm_seed[i].position) * noisegen.arm_scale[i] * mag * limit(time,noisegen.global_seed) .* bias
            noisegen.arm_noise[i].rotation = noise3D(noisegen.time,noisegen.arm_seed[i].rotation) * noisegen.arm_scale[i] * 0.3 * mag
        end
    end
    # Handle Direct Control
    for i=1:noisegen.num_dc
        if noisegen.dc_scale[i] > 0.0 && noisegen.dc_weight[i] > 0
            noisegen.dc_noise[i] = noise(noisegen.time,noisegen.dc_seed[i]) * noisegen.dc_scale[i]
        end
    end
    # Handle fixed frame noise
    if noisegen.base_scale > 0.0
        noisegen.base_noise.position = noise3D(noisegen.time,noisegen.base_seed.position) * noisegen.base_scale * mag .* bias
        #noisegen.base_noise.rotation = noise3D(noisegen.time,noisegen.base_seed.rotation) * noisegen.base_scale * 0.05 * mag
        #noisegen.base_noise.position[1] *= 0
        #noisegen.base_noise.position[3] *= 0
    end

end
