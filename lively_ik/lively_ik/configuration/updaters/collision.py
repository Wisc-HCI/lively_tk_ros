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

def derive_nn_main(config):
    if not config['train_directive']:
        return {'intercepts':[],'coefs':[],'split_point':None}
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
    split_point = find_optimal_split_point(clf_main,config['robot'],
                                           config['base_link_motion_bounds'],
                                           config['collision_graph'],
                                           2000,jointpoint=False)

    intercepts = recursive_tolist(clf_main.intercepts_)
    coefs = recursive_tolist(clf_main.coefs_)

    return {'intercepts':intercepts,'coefs':coefs,'split_point':split_point}

def derive_nn_jointpoint(config):
    if not config['train_directive']:
        return {'intercepts':[],'coefs':[],'split_point':None}
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
    split_point = find_optimal_split_point(clf_jp,config['robot'],
                                           config['base_link_motion_bounds'],
                                           config['collision_graph'],
                                           2000,jointpoint=True)

    intercepts = recursive_tolist(clf_jp.intercepts_)
    coefs = recursive_tolist(clf_jp.coefs_)

    return {'intercepts':intercepts,'coefs':coefs,'split_point':split_point}

def derive_training_scores(config):
    if config['robot'] == None or not config['train_directive']:
        return []
    scores = []
    for sample in config['training_samples']:
        frames = config['robot'].getFrames(sample[0:3],sample[3:])
        scores.append(config['collision_graph'].get_collision_score(sample[0:3],frames))
        # print(f'State: {sample[0:3]+sample[3:]} ({len(sample)}) Score: {scores[-1]}')
    return scores

def derive_collision_graph(config):
    if config['robot'] == None or not config['train_directive']:
        return None
    return CollisionGraph(config, config['robot'], sample_states=config['states'])

def derive_training_frames(config):
    if config['robot'] == None or not config['train_directive']:
        return []
    training_frames = []
    for sample in config['training_samples']:
        frames = config['robot'].getFrames(sample[0:3],sample[3:])
        training_frames.append(frames_to_jt_pt_vec(frames))
    return training_frames

def derive_training_samples(config):
    if config['robot'] == None or not config['train_directive']:
        return []
    samples = []
    for i in range(200000):
        try:
            state = config['states'][i][0]+config['states'][i][1]
        except:
            state = []
            for dim in [0,1,2]:
                rand = nprandom.uniform(low=config['base_link_motion_bounds'][dim][0],
                                        high=config['base_link_motion_bounds'][dim][1], size=(1))[0]
                state.append(rand)
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

def find_optimal_split_point(clf,robot,base_bounds,graph,num_samples=2000,jointpoint=False):
    predictions = []
    ground_truths = []
    for i in range(num_samples):
        state = []
        for b in base_bounds:
            rand = nprandom.uniform(low=b[0], high=b[1], size=(1))[0]
            state.append(rand)
        for b in robot.bounds:
            rand = nprandom.uniform(low=b[0], high=b[1], size=(1))[0]
            state.append(rand)
        if jointpoint:
            frames = robot.getFrames(state[0:3],state[3:])
            jt_pt_vec = frames_to_jt_pt_vec(frames)
            pred = clf.predict([jt_pt_vec])
        else:
            pred = clf.predict([state])
        ground_truth = graph.get_collision_score_of_state(state[0:3],state[3:])
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
