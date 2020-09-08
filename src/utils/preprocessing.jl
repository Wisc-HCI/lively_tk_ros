#!/usr/bin/env julia
using LivelyIK
using Random

function state_to_joint_pts(x, vars)
    # return x
    joint_pts = []
    for i=1:vars.robot.num_chains
        vars.robot.arms[i].getFrames(x[vars.robot.subchain_indices[i]])
    end

    for j=1:vars.robot.num_chains
        out_pts = vars.robot.arms[j].out_pts
        for k = 1:length(out_pts)
            for l=1:3
                push!(joint_pts, out_pts[k][l])
            end
        end
    end

    return joint_pts
end

loss(w,x,y) = Knet.mean(abs2, y - predict(w,x) )
loss2(w,x,y) = Knet.mean(abs2, y - predict(w,x))[1]
lossgradient = Knet.grad(loss)

function total_loss(w, all_x, all_y)
    sum = 0.0
    for i = 1:length(all_x)
        sum += loss(w, all_x[i], all_y[i])
    end
    return sum
end

function total_loss2(w, all_x, all_y)
    sum = 0.0
    for i = 1:length(all_x)
        sum += loss2(w, all_x[i], all_y[i])
    end
    return sum
end

function train(model, data, optim)
    for (x,y) in data
        grads = lossgradient(model,x,y)
        Knet.update!(model, grads, optim)
    end
end

function get_rand_state_with_bounds(bounds)
    sample = []
    for b in bounds
        push!(sample, rand(Uniform(b[1], b[2])))
    end
    return sample
end

function shuffle_ins_and_outs(ins, outs, states)
    new_ins = []
    new_outs = []
    new_states = []

    idxs = 1:length(ins)
    shuffled_idxs = shuffle(idxs)

    for i=1:length(shuffled_idxs)
        push!(new_ins, ins[shuffled_idxs[i]])
        push!(new_outs, outs[shuffled_idxs[i]])
        push!(new_states, states[shuffled_idxs[i]])
    end

    return new_ins, new_outs, new_states
end

function get_batched_data(ins, outs, batch_size)
    batch = Knet.minibatch(ins, outs, batch_size)
    batches = []

    for (index, value) in enumerate(batch)
        push!(batches, value)
    end

    batched_data = []
    for batch_idx = 1:length(batches)
        push!(batched_data, [])
        for i = 1:length(batches[batch_idx][1])
            in = batches[batch_idx][1][i]
            out = batches[batch_idx][2][i]
            push!( batched_data[batch_idx], (in, out) )
        end
    end

    return batched_data
end

function preprocess(info, rcl_node, cb)
    collision_transfer = pyimport("lively_ik.utils.collision_transfer")
    lively_ik = pyimport("lively_ik")

    relaxedIK = LivelyIK.get_standard(info, rcl_node; preconfigured=true)
    cv = collision_transfer.CollisionVars(info, rcl_node)

    num_dof = relaxedIK.relaxedIK_vars.robot.num_dof
    state_to_joint_pts_closure = (x) -> state_to_joint_pts(x, relaxedIK.relaxedIK_vars)

    # Create data ##################################################################
    num_samples = 200000
    in_states = []
    out_scores = []
    states = []
    test_ins = []
    test_outs = []
    test_states = []

    for i=1:num_samples
        state = get_rand_state_with_bounds(relaxedIK.relaxedIK_vars.vars.bounds)
        score = [collision_transfer.get_score(state, cv)]
        push!(states, state)
        push!(in_states, state_to_joint_pts_closure(state))
        push!(out_scores, score)
        # TODO: Emit to socket with state update
        cb("julia",i/num_samples*10)
        # println("sample $i of $num_samples ::: state: $in, y: $out")
    end

    training_states = info["training_states"]
    num_samples = length(training_states)
    for i=1:num_samples
        state = training_states[i]
        score = [collision_transfer.get_score(state, cv)]
        push!(states, state)
        push!(in_states, state_to_joint_pts_closure(state))
        push!(out_scores, score)
        # TODO: Emit to socket with state update
        cb("julia",i/num_samples*10+10)
        #println("manual sample $i of $num_samples ::: state: $in, y: $out")
    end

    problem_states = info["problem_states"]
    num_samples = length(problem_states)
    length_of_sample = length(problem_states[1])
    num_rands_per = 50
    for i=1:num_samples
        for j = 1:num_rands_per
            r = rand(Uniform(-.005,.005), length_of_sample)
            state = problem_states[i] + r
            score = [collision_transfer.get_score(state, cv)]
            push!(states, state)
            push!(in_states, state_to_joint_pts_closure(state))
            push!(out_scores, score)
            # TODO: Emit to socket with state update
            cb("julia",i/num_samples*10+20)
            #println("problem state sample $i ($j / $num_rands_per) of $num_samples ::: state: $in, y: $out")
        end
    end

    sample_states = info["sample_states"]
    num_samples = length(sample_states)
    length_of_sample = length(sample_states[1])
    num_rands_per = 50
    for i=1:num_samples
        for j = 1:num_rands_per
            r = rand(Uniform(-.005,.005), length_of_sample)
            state = sample_states[i] + r
            score = [collision_transfer.get_score(state, cv)]
            push!(states, state)
            push!(in_states, state_to_joint_pts_closure(state))
            push!(out_scores, score)
            # TODO: Emit to socket with state update
            cb("julia",i/num_samples*10+30)
            #println("sample state sample $i ($j / $num_rands_per) of $num_samples ::: state: $in, y: $out")
        end
    end

    for i=1:50
        # in = rand(Uniform(-6,6), num_dof)
        state = get_rand_state_with_bounds(relaxedIK.relaxedIK_vars.vars.bounds)
        score = [collision_transfer.get_score(state, cv)]
        push!(test_states, state)
        push!(test_ins, state_to_joint_pts_closure(state))
        push!(test_outs, score)
    end

    # Make batches #################################################################
    batch = Knet.minibatch(in_states, out_scores, 200)
    batches = []

    for (index, value) in enumerate(batch)
        push!(batches, value)
    end

    # Finalize data ################################################################
    batched_data = []
    data = []
    test_data = []

    for batch_idx = 1:length(batches)
        push!(batched_data, [])
        for i = 1:length(batches[batch_idx][1])
            state = batches[batch_idx][1][i]
            score = batches[batch_idx][2][i]
            push!( batched_data[batch_idx], (state, score) )
        end
    end

    for i = 1:length(in_states)
        push!( data, (in_states[i], out_scores[i]) )
    end

    for i = 1:length(test_ins)
        push!( test_data, (test_ins[i], test_outs[i]) )
    end

    net_width = 18
    rand_val = 1.0
    w1 = [ rand_val*Knet.xavier(net_width, length(in_states[1]) ), zeros(Float64,net_width,1),
        rand_val*Knet.xavier(net_width, net_width), zeros(Float64,net_width,1),
        rand_val*Knet.xavier(net_width, net_width), zeros(Float64,net_width,1),
        rand_val*Knet.xavier(net_width, net_width), zeros(Float64,net_width,1),
        rand_val*Knet.xavier(1, net_width), zeros(Float64,1,1)  ]


    net_width = 26
    rand_val = 1.0
    w2 = [ rand_val*Knet.xavier(net_width, length(in_states[1]) ), zeros(Float64,net_width,1),
        rand_val*Knet.xavier(net_width, net_width), zeros(Float64,net_width,1),
        rand_val*Knet.xavier(net_width, net_width), zeros(Float64,net_width,1),
        rand_val*Knet.xavier(net_width, net_width), zeros(Float64,net_width,1),
        rand_val*Knet.xavier(1, net_width), zeros(Float64,1,1)  ]


    net_width = 34
    rand_val = 1.0
    w3 = [ rand_val*Knet.xavier(net_width, length(in_states[1]) ), zeros(Float64,net_width,1),
        rand_val*Knet.xavier(net_width, net_width), zeros(Float64,net_width,1),
        rand_val*Knet.xavier(net_width, net_width), zeros(Float64,net_width,1),
        rand_val*Knet.xavier(net_width, net_width), zeros(Float64,net_width,1),
        rand_val*Knet.xavier(1, net_width), zeros(Float64,1,1)  ]

    # Optimize #####################################################################
    o1 = optimizers(w1, Knet.Adam)
    o2 = optimizers(w2, Knet.Adam)
    o3 = optimizers(w3, Knet.Adam)

    tl = total_loss2(w1, test_ins, test_outs)
    tl_train = total_loss( w1, in_states, out_scores )
    num_epochs = 2
    batch_size = 200
    best_w = []
    best_score = 10000000000000000.0
    improve_idx = 1

    for epoch=1:num_epochs
        # shuffle data here...get new batched data
        new_ins, new_outs, new_states = shuffle_ins_and_outs(in_states, out_scores, states)
        batched_data = get_batched_data(new_ins, new_outs, batch_size)
        for b = 1:length(batched_data)
            train(w1, batched_data[b], o1)
            train(w2, batched_data[b], o2)
            train(w3, batched_data[b], o3)
            cb("julia",(((epoch-1)*length(batched_data)+b)/(num_epochs*length(batched_data)))*50+40)
        end
        tl = total_loss2(w1, test_ins, test_outs)
        tl_train = total_loss( w1, new_ins, new_outs )
    end

    @save lively_ik.BASE * "/config/collision_nn/" * info["robot_name"] * "_1" w1
    @save lively_ik.SRC * "/config/collision_nn/" * info["robot_name"] * "_1" w1
    @save lively_ik.BASE * "/config/collision_nn/" * info["robot_name"] * "_2" w2
    @save lively_ik.SRC * "/config/collision_nn/" * info["robot_name"] * "_2" w2
    @save lively_ik.BASE * "/config/collision_nn/" * info["robot_name"] * "_3" w3
    @save lively_ik.SRC * "/config/collision_nn/" * info["robot_name"] * "_3" w3

    w1 = BSON.load(lively_ik.BASE * "/config/collision_nn/" * info["robot_name"] * "_1")[:w1]
    model1 = (x) -> predict(w1, x)[1]
    w2 = BSON.load(lively_ik.BASE * "/config/collision_nn/" * info["robot_name"] * "_2")[:w2]
    model2 = (x) -> predict(w2, x)[1]
    w3 = BSON.load(lively_ik.BASE * "/config/collision_nn/" * info["robot_name"] * "_3")[:w3]
    model3 = (x) -> predict(w3, x)[1]

    ################################################################################

    # get t, c, and f values #######################################################
    t_val1, c_val1, f_val1 = get_t_c_and_f_values(w1, cv, relaxedIK)
    t_val2, c_val2, f_val2 = get_t_c_and_f_values(w2, cv, relaxedIK)
    t_val3, c_val3, f_val3 = get_t_c_and_f_values(w3, cv, relaxedIK)

    model1_acc = 0.0
    model2_acc = 0.0
    model3_acc = 0.0
    total_acc_count = 2000

    for i = 1:total_acc_count
        r = get_rand_state_with_bounds(relaxedIK.relaxedIK_vars.vars.bounds)
        state = state_to_joint_pts_withreturn(r, relaxedIK.relaxedIK_vars)
        model1_score = model1(state)
        model2_score = model2(state)
        model3_score = model3(state)
        ground_truth_score = collision_transfer.get_score(r, cv)
        thresh = 1.0
        if ground_truth_score >= 5.0
            if model1_score >= thresh
                model1_acc += 1.
            end

            if model2_score >= thresh
                model2_acc += 1.
            end

            if model3_score >= thresh
                model3_acc += 1.
            end
        else
            if model1_score < thresh
                model1_acc += 1.
            end

            if model2_score < thresh
                model2_acc += 1.
            end

            if model3_score < thresh
                model3_acc += 1.
            end
        end
        cb("julia",i/total_acc_count*9+90)
    end

    model1_acc = model1_acc/total_acc_count
    model2_acc = model2_acc/total_acc_count
    model3_acc = model3_acc/total_acc_count

    sorted = sortperm([model1_acc, model2_acc, model3_acc])
    # println("model 1 accuracy: $model1_acc, model 2 accuracy: $model2_acc, model 3 accuracy: $model3_acc")

    fp = open(lively_ik.BASE * "/config/collision_nn/" * info["robot_name"] * "_params_1", "w")
    write(fp, "$t_val1, $c_val1, $f_val1")
    close(fp)

    fp = open(lively_ik.SRC * "/config/collision_nn/" * info["robot_name"] * "_params_1", "w")
    write(fp, "$t_val1, $c_val1, $f_val1")
    close(fp)

    fp = open(lively_ik.BASE * "/config/collision_nn/" * info["robot_name"] * "_params_2", "w")
    write(fp, "$t_val2, $c_val2, $f_val2")
    close(fp)

    fp = open(lively_ik.SRC * "/config/collision_nn/" * info["robot_name"] * "_params_2", "w")
    write(fp, "$t_val2, $c_val2, $f_val2")
    close(fp)

    fp = open(lively_ik.BASE * "/config/collision_nn/" * info["robot_name"] * "_params_3", "w")
    write(fp, "$t_val3, $c_val3, $f_val3")
    close(fp)

    fp = open(lively_ik.SRC * "/config/collision_nn/" * info["robot_name"] * "_params_3", "w")
    write(fp, "$t_val3, $c_val3, $f_val3")
    close(fp)

    fp = open(lively_ik.BASE * "/config/collision_nn/" * info["robot_name"] * "_network_rank", "w")
    write(fp, "$(sorted[3]), $(sorted[2]), $(sorted[1])")
    close(fp)

    fp = open(lively_ik.SRC * "/config/collision_nn/" * info["robot_name"] * "_network_rank", "w")
    write(fp, "$(sorted[3]), $(sorted[2]), $(sorted[1])")
    close(fp)
    cb("julia",100)
end
