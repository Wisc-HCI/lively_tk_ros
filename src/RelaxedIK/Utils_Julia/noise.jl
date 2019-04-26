mutable struct NoiseGenerator
    arms
    time
end

function NoiseGenerator(weights, mask)

    n = NoiseGenerator([], 0.0)

    update!(n, 0)

    return n
end

function update!(noisegen, time)
    noisegen.time = time
    for i=1:length(arms)
        
    end
end
