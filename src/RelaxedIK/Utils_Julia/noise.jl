include("noise_utils.jl")

# Remove in final
using RobotOS
@rosimport geometry_msgs.msg: Point, Quaternion, Pose

mutable struct posenoise
    position
    rotation
end

mutable struct NoiseGenerator
    arms
    scale
    seeds
    time
    # Remove in final
    pub
end

function NoiseGenerator(scale, base_link_noise)

    push!(scale,base_link_noise)
    arms = []
    seeds = []

    for i=1:length(scale)
        push!(arms, posenoise(zeros(3),zeros(3)))
        push!(seeds, posenoise(10*rand(3),10*rand(3)))
    end

    pub = Publisher("/lively_ik/noise", Pose, queue_size = 3)

    # n = NoiseGenerator(arms, scale, seeds, 0.0)

    # Remove in final
    n = NoiseGenerator(arms, scale, seeds, 0.0, pub)

    update!(n, 0.0, 0.0)

    return n
end

function update!(noisegen, time, priority)
    # Priority is a value 0-1.
    # Higher priority means less noise
    scale = 1-priority
    noisegen.time = time
    for i=1:length(noisegen.scale)
        if noisegen.scale[i] > 0.0
            noisegen.arms[i].position = noise3D(noisegen.time,noisegen.seeds[i].position) * noisegen.scale[i] * scale
            noisegen.arms[i].rotation = noise3D(noisegen.time,noisegen.seeds[i].rotation) * noisegen.scale[i] * 0.3 * scale
        end
    end

    pose = Pose()
    pose.position.x = noisegen.arms[1].position[1]
    pose.position.y = noisegen.arms[1].position[2]
    pose.position.z = noisegen.arms[1].position[3]
    # pose.orientation.w = noisegen.arms[1].rotation[1]
    pose.orientation.x = noisegen.arms[1].rotation[1]
    pose.orientation.y = noisegen.arms[1].rotation[2]
    pose.orientation.z = noisegen.arms[1].rotation[3]
    publish(noisegen.pub,pose)

    # println(temp_scale,": ",noisegen.arms[1])
end
