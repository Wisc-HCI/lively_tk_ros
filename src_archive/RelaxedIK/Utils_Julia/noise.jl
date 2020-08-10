include("noise_utils.jl")

# Remove in final
using RobotOS
@rosimport geometry_msgs.msg: Point, Quaternion, Pose

mutable struct scalarNoise
    scale
    frequency
    seed
    value
end

mutable struct vectorNoise
    scale
    frequency
    seed
    value
end

mutable struct voidNoise
    value
end

mutable struct NoiseGenerator
    generators
    base
end

function scalarNoise(scale,frequency)
    return scalarNoise(scale, frequency, 1000*rand(), 0)
end

function vectorNoise(scale,frequency)
    return vectorNoise(scale, frequency, 1000*rand(3), zeros(3))
end

function voidNoise()
    return voidNoise(0)
end

function update!(scalarnoise::scalarNoise,time,bias)
    if scalarnoise.scale > 0
        scalarnoise.value = noise1D(time,scalarnoise.seed,scalarnoise.frequency) * scalarnoise.scale
    end
end

function update!(vectornoise::vectorNoise,time,bias)
    if vectornoise.scale > 0
        vectornoise.value = noise3D(time,vectornoise.seed,vectornoise.frequency) * vectornoise.scale .* bias
    end
end

function update!(voidnoise::voidNoise,time,bias)

end

function NoiseGenerator(objective_info,base_scale,base_freq)

    generators = []
    base = vectorNoise(base_scale,base_freq)

    for i in 1:length(objective_info)
        if objective_info[i]["type"] == "positional_noise" || objective_info[i]["type"] == "rotational_noise"
            push!(generators, vectorNoise(objective_info[i]["scale"],objective_info[i]["frequency"]))
        elseif objective_info[i]["type"] == "dc_noise"
            push!(generators, scalarNoise(objective_info[i]["scale"],objective_info[i]["frequency"]))
        else
            push!(generators, voidNoise())
        end
    end

    n = NoiseGenerator(generators, base)
    return n

    # end
    #
    # num_dc = length(dc_scale)
    #
    # arm_noise = []
    # dc_noise  = zeros(num_dc)
    # arm_seed  = []
    # dc_seed   = []
    #
    # for i=1:num_chains
    #     push!(arm_noise, posenoise(zeros(3),zeros(3)))
    #     push!(arm_seed,  posenoise(1000*rand(3),1000*rand(3)))
    # end
    # for i=1:num_dc
    #     push!(dc_seed,  1000*rand())
    # end
    #
    # #dc_mask = zeros(length(dc_names))
    #
    # base_noise = posenoise(zeros(3),zeros(3))
    # base_seed  = posenoise(1000*rand(3),1000*rand(3))
    # global_seed = rand()*1000
    #
    # # Remove in final

end

function limit(time,seed)
    raw = noise1D(time,seed,4)
    limit = 1/(1+exp(5-10*abs(raw)))
    return limit
end

function update!(noisegen, time, bias)
    update!(noisegen.base,time,bias)
    for i in 1:length(noisegen.generators)
        update!(noisegen.generators[i],time,bias)
    end
    # noisegen.time = time
    #
    # # Handle End Effector Noise
    # for i=1:noisegen.num_chains
    #     if noisegen.arm_position_scale[i] > 0.0 || noisegen.arm_rotation_scale[i] > 0.0
    #         # println(["Before pos",noisegen.arm_position_scale[i],noisegen.arm_position_freq[i]])
    #         noisegen.arm_noise[i].position = noise3D(noisegen.time,noisegen.arm_seed[i].position,noisegen.arm_position_freq[i]) * noisegen.arm_position_scale[i] .* bias# * limit(time,noisegen.global_seed)
    #         # println(["Before rot",noisegen.arm_position_scale[i],noisegen.arm_position_freq[i]])
    #         noisegen.arm_noise[i].rotation = noise3D(noisegen.time,noisegen.arm_seed[i].rotation,noisegen.arm_rotation_freq[i]) * noisegen.arm_rotation_scale[i]
    #     end
    # end
    # # Handle Direct Control
    # for i=1:noisegen.num_dc
    #     if noisegen.dc_scale[i] > 0.0
    #         noisegen.dc_noise[i] = noise1D(noisegen.time,noisegen.dc_seed[i],noisegen.dc_freq[i]) * noisegen.dc_scale[i]
    #     end
    # end
    # # Handle fixed frame noise
    # if noisegen.base_scale > 0.0
    #     noisegen.base_noise.position = noise3D(noisegen.time,noisegen.base_seed.position,noisegen.base_freq) * noisegen.base_scale .* bias
    #     #noisegen.base_noise.rotation = noise3D(noisegen.time,noisegen.base_seed.rotation) * noisegen.base_scale * 0.05 * mag
    #     #noisegen.base_noise.position[1] *= 0
    #     #noisegen.base_noise.position[3] *= 0
    # end

end
