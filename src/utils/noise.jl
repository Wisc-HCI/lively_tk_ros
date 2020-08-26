
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
end
