include("noise_utils.jl")

mutable struct posrot
    position
    rotation
end

mutable struct NoiseGenerator
    arms
    scale
    seeds
    time
end

function NoiseGenerator(scale, base_link_noise)

    push!(scale,base_link_noise)
    arms = []
    seeds = []

    for i=1:length(scale)
        push!(arms, posrot(zeros(3),zeros(4)))
        push!(seeds, posrot(rand(3),rand(4)))
    end

    n = NoiseGenerator(arms, scale, seeds, 0.0)

    update!(n, 0.0, 0.0)

    return n
end

function update!(noisegen, wait, time)
    # Wait is transformed to a value 0-1,
    # based on the amount of time that has elapsed since the last true
    # solve request to lively_ik. This causes the amount of noise to
    # slowly ramp up.
    temp_scale = 2 ^ (0.05 * wait - 10) / (2 ^ (0.05 * wait - 10) + 1)
    noisegen.time = time
    for i=1:length(noisegen.scale)
        if noisegen.scale[i] > 0.0
            noisegen.arms[i].position = noise3D(noisegen.time,noisegen.seeds[i].position) * noisegen.scale[i] * temp_scale
            noisegen.arms[i].rotation = noise4D(noisegen.time,noisegen.seeds[i].rotation) * noisegen.scale[i] * 0.01 * temp_scale
        end
    end
    # println(temp_scale,": ",noisegen.arms[1])
end
