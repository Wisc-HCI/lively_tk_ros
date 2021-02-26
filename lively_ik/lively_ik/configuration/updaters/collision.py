from lively_ik.configuration.collision_graph import CollisionGraph
from sklearn.neural_network import MLPClassifier, MLPRegressor
import numpy.random as nprandom
import numpy as np

def recursive_tolist(obj):
    if isinstance(obj,np.ndarray):
        return obj.tolist()
    elif isinstance(obj,list):
        return [recursive_tolist(o) for o in obj]
    else:
        return obj

def derive_nn_progress(config):
    statuses = [
        config['valid_nn'],
        config['nn_main'] != {'intercepts':[],'coefs':[],'split_point':None},
        config['nn_jointpoint'] != {'intercepts':[],'coefs':[],'split_point':None},
        len(config['training_scores']) >= 200000,
        len(config['training_frames']) >= 200000,
        len(config['training_samples']) >= 200000,
        config['collision_graph'] != None
    ]
    # print(statuses)
    percent = int(sum(statuses) / float(len(statuses)) * 100)
    return percent

def derive_nn_main(config):
    layer_width = 15
    num_layers = 5

    # Train Main NN
    hls = []
    for i in range(num_layers):
        hls.append(layer_width)
    hls = tuple(hls)
    clf_main = MLPRegressor(solver='adam', alpha=1,
                            hidden_layer_sizes=hls, max_iter=300000, verbose=True,
                            learning_rate='adaptive')

    clf_main.fit(config['training_samples'], config['training_scores'])
    split_point = find_optimal_split_point(clf_main,config['robot'],config['collision_graph'],2000,jointpoint=False)

    intercepts = recursive_tolist(clf_main.intercepts_)
    coefs = recursive_tolist(clf_main.coefs_)

    return {'intercepts':intercepts,'coefs':coefs,'split_point':split_point}

def derive_nn_jointpoint(config):
    layer_width = 15
    num_layers = 5

    # Train Jointpoint NN
    hls = []
    for i in range(num_layers):
        hls.append(layer_width)
    hls = tuple(hls)
    clf_jp = MLPRegressor(solver='adam', alpha=1,
                            hidden_layer_sizes=hls, max_iter=300000, verbose=True,
                            learning_rate='adaptive')

    clf_jp.fit(config['training_frames'], config['training_scores'])
    split_point = find_optimal_split_point(clf_jp,config['robot'],config['collision_graph'],2000,jointpoint=True)

    intercepts = recursive_tolist(clf_jp.intercepts_)
    coefs = recursive_tolist(clf_jp.coefs_)

    return {'intercepts':intercepts,'coefs':coefs,'split_point':split_point}

def derive_training_scores(config):
    scores = []
    for sample in config['training_samples']:
        frames = config['robot'].getFrames(sample)
        scores.append(config['collision_graph'].get_collision_score(frames))
    return scores

def derive_collision_graph(config):
    return CollisionGraph(config, config['robot'], sample_states=config['states'])

def derive_training_frames(config):
    training_frames = []
    for sample in config['training_samples']:
        frames = config['robot'].getFrames(sample)
        training_frames.append(frames_to_jt_pt_vec(frames))
    return training_frames

def derive_training_samples(config):
    samples = []
    for i in range(200000):
        try:
            state = config['states'][i]
        except:
            state = []
            for b in config['robot'].bounds:
                rand = nprandom.uniform(low=b[0], high=b[1], size=(1))[0]
                state.append(rand)
        samples.append(state)
    return samples

def frames_to_jt_pt_vec(all_frames):
    out_vec = []
    for frames in all_frames:
        jt_pts = frames[0]
        for j in jt_pts:
            out_vec.append(j[0])
            out_vec.append(j[1])
            out_vec.append(j[2])
    return out_vec

def find_optimal_split_point(clf,robot,graph,num_samples=2000,jointpoint=False):
    predictions = []
    ground_truths = []
    for i in range(num_samples):
        if jointpoint:
            state = []
            for b in robot.bounds:
                rand = nprandom.uniform(low=b[0], high=b[1], size=(1))[0]
                state.append(rand)
            frames = robot.getFrames(state)
            jt_pt_vec = frames_to_jt_pt_vec(frames)
            pred = clf.predict([jt_pt_vec])
        else:
            state = []
            for b in robot.bounds:
                rand = nprandom.uniform(low=b[0], high=b[1], size=(1))[0]
                state.append(rand)
            pred = clf.predict([state])
        ground_truth = graph.get_collision_score_of_state(state)
        predictions.append(pred)
        ground_truths.append(ground_truth)
    best_split = 5.0
    best_split_score = 100000000000.
    split_point = 8.0
    best_false_positive_count = 0.0
    best_false_negative_count = 0.0
    while split_point > 0.0:
        false_positive_count = 0.
        false_negative_count = 0.
        total_count = 0.
        for i in range(num_samples):
            if predictions[i] >= split_point and ground_truths[i] < 5.0:
                false_negative_count += 1.0
            elif predictions[i] < split_point and ground_truths[i] >= 5.0:
                false_positive_count += 1.0
            total_count += 1.0
        split_score = 2.0*(false_positive_count / total_count) + (false_negative_count / total_count)
        if split_score < best_split_score:
            best_split_score = split_score
            best_split = split_point
            best_false_negative_count = false_negative_count
            best_false_positive_count = false_positive_count
        split_point -= 0.01
    return best_split
