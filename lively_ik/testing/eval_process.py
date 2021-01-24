import yaml
import os
from julia import LivelyIK
import random
from lively_ik import SRC, BASE, INFO_PARAMS, get_configs
from lively_ik.testing.eval import NaoStaticValence, NaoWaveAction, PandaStaticValence, PandaPickupAction, NaoProfiler, PandaProfiler
from lively_ik.groove.relaxed_ik_container import RelaxedIKContainer
from lively_ik.utils.colors import bcolors
from std_msgs.msg import String, Float64, Int16
from wisc_actions.elements import Pose, Position, Orientation, ModeTrajectory, PoseTrajectory
from wisc_msgs.msg import GoalUpdate, GoalUpdates, LivelyGoals, EvalResult
from wisc_msgs.srv import UpdateGoals
import time
import json
import pandas
import numpy as np
import subprocess

ALLOWED_NAIVE_JOINTS = {
                    'ur5_robotiq_85':['elbow_joint','shoulder_lift_joint','shoulder_pan_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint','gripper_finger1_joint','gripper_finger2_joint'],
                    'panda':['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7','panda_finger_joint1','panda_finger_joint2'],
                    'nao_v5':['HeadPitch','HeadYaw','LElbowRoll','LElbowYaw','LHand','LShoulderPitch','LShoulderRoll','LWristYaw','RElbowRoll','RElbowYaw','RHand','RShoulderPitch','RShoulderRoll','RWristYaw']
                    }

def values_for_joint(config,joint,data,group):
    idx = config['joint_ordering'].index(joint)
    values = []
    for step in data:
        values.append(step[group][idx])
    return values

def clamp(x, lo, hi):
    if lo <= x <= hi:
        return x
    elif x < lo:
        return lo
    elif x > hi:
        return hi

def evaluate(evaluation,evals,joint_mags,panda_collision,nao_v5_collision):
    print('\n'+bcolors.OKBLUE+evaluation+bcolors.ENDC)
    evals[evaluation]['test'].generate()

    fn = 'julia {0}/lively_ik/EvalNode.jl {0}/config/info_files/{1}.yaml {0}/eval/{2}.json'.format(SRC,evals[evaluation]['config']['robot_name'],evaluation)
    subprocess.run(fn,shell=True)
    with open(SRC+'/eval/'+evaluation+'_sol.json','r') as stream:
        data = json.load(stream)
    allowed_joints = ALLOWED_NAIVE_JOINTS[evals[evaluation]['config']['robot_name']]
    joint_ordering = evals[evaluation]['config']['joint_ordering']
    joint_limits = evals[evaluation]['config']['joint_limits']
    df_data = {col:[] for col in ['time','condition','collision']+joint_ordering}
    seeds = [random.random()*1000 for j in joint_ordering]
    joint_mag = joint_mags[evals[evaluation]['config']['robot_name']]
    if evaluation == 'panda_pickup_task':
        noise_strength = [
        {'time':0,'mode':1},
        {'time':265,'mode':1},
        {'time':269,'mode':0},
        {'time':289,'mode':0},
        {'time':291,'mode':1},
        ]
        noise_trajectory = ModeTrajectory(noise_strength,method='interp1d',kind='cubic')
    for step in data:
        time = step['time']
        df_data['time'].extend(4*[time])
        df_data['condition'].extend(['lively','relaxed','dc','naive'])
        if evaluation == 'panda_pickup_task':
            amp = noise_trajectory[time]
        else:
            amp = 1

        # print('{0}: {1}'.format(time,amp))
        naive = []
        for idx,joint in enumerate(joint_ordering):
            df_data[joint].append(step['lik_sol'][idx])
            df_data[joint].append(step['rik_sol'][idx])
            df_data[joint].append(step['dc'][idx])
            value = step['rik_sol'][idx] + amp * LivelyIK.noise1D(time,seeds[idx],evals[evaluation]['frequency']) * joint_mag[joint]
            value = clamp(value,evals[evaluation]['config']['joint_limits'][idx][0],evals[evaluation]['config']['joint_limits'][idx][1])
            naive.append(value)
            df_data[joint].append(value)

        if 'panda' in evaluation:
            df_data['collision'].append(panda_collision(step['lik_sol']))
            df_data['collision'].append(panda_collision(step['rik_sol']))
            df_data['collision'].append(panda_collision(step['dc']))
            df_data['collision'].append(panda_collision(naive))
        else:
            df_data['collision'].append(nao_v5_collision(step['lik_sol']))
            df_data['collision'].append(nao_v5_collision(step['rik_sol']))
            df_data['collision'].append(nao_v5_collision(step['dc']))
            df_data['collision'].append(nao_v5_collision(naive))
    df = pandas.DataFrame(df_data)
    df.to_csv(SRC+'/eval/'+evaluation+'.csv')

def main():
    configs = get_configs()
    panda_container = RelaxedIKContainer(configs['panda'])
    nao_v5_container = RelaxedIKContainer(configs['nao_v5'])

    panda_collision = lambda angles: panda_container.collision_graph.get_collision_score_of_state(angles)
    nao_v5_collision = lambda angles: nao_v5_container.collision_graph.get_collision_score_of_state(angles)

    evals = {
    'panda_profiler':{'frequency':5,'config':configs['panda'],'test':PandaProfiler(5)},
    'nao_v5_profiler':{'frequency':5,'config':configs['nao_v5'],'test':NaoProfiler(5)},
    # 'nao_static_weights_negative':{'frequency':10,'config':configs['nao_v5'],'test':NaoStaticValence('negative')},
    # 'nao_static_weights_positive':{'frequency':3,'config':configs['nao_v5'],'test':NaoStaticValence('positive')},
    # 'nao_wave_task':{'frequency':3,'config':configs['nao_v5'],'test':NaoWaveAction()},
    # 'panda_static_weights_negative':{'frequency':10,'config':configs['panda'],'test':PandaStaticValence('negative')},
    # 'panda_static_weights_positive':{'frequency':3,'config':configs['panda'],'test':PandaStaticValence('positive')},
    # 'panda_pickup_task':{'frequency':5,'config':configs['panda'],'test':PandaPickupAction()},
    }

    # Generate joint magnitudes
    joint_mags = {
        'panda':{jn:0 for jn in configs['panda']['joint_ordering']},
        'nao_v5':{jn:0 for jn in configs['nao_v5']['joint_ordering']}
    }

    try:
        panda_profile = pandas.read_csv(SRC+'/eval/panda_profiler.csv')
    except:
        for jn in configs['panda']['joint_ordering']:
            joint_mags['panda'][jn] = 0
        evaluate('panda_profiler',evals,joint_mags,panda_collision,nao_v5_collision)
        panda_profile = pandas.read_csv(SRC+'/eval/panda_profiler.csv')

    try:
        nao_v5_profile = pandas.read_csv(SRC+'/eval/nao_v5_profiler.csv')
    except:
        for jn in configs['nao_v5']['joint_ordering']:
            joint_mags['nao_v5'][jn] = 0
        evaluate('nao_v5_profiler',evals,joint_mags,panda_collision,nao_v5_collision)
        nao_v5_profile = pandas.read_csv(SRC+'/eval/nao_v5_profiler.csv')

    panda_rik_profile = panda_profile[panda_profile['condition']=='relaxed'].set_index('time')
    panda_lik_profile = panda_profile[panda_profile['condition']=='lively'].set_index('time')
    for jn in configs['panda']['joint_ordering']:
        if jn in ALLOWED_NAIVE_JOINTS['panda']:
            joint_mags['panda'][jn] = np.std(panda_lik_profile[jn]-panda_rik_profile[jn])#*0.2
        else:
            joint_mags['panda'][jn] = 0

    nao_v5_rik_profile = nao_v5_profile[nao_v5_profile['condition']=='relaxed'].set_index('time')
    nao_v5_lik_profile = nao_v5_profile[nao_v5_profile['condition']=='lively'].set_index('time')
    for jn in configs['nao_v5']['joint_ordering']:
        if jn in ALLOWED_NAIVE_JOINTS['nao_v5']:
            joint_mags['nao_v5'][jn] = np.std(nao_v5_lik_profile[jn]-nao_v5_rik_profile[jn])#*1.2
        else:
            joint_mags['nao_v5'][jn] = 0

    del panda_profile
    del nao_v5_profile
    del panda_lik_profile
    del nao_v5_lik_profile
    del panda_rik_profile
    del nao_v5_rik_profile

    for evaluation in evals:
        if 'profile' not in evaluation:
            evaluate(evaluation,evals,joint_mags,panda_collision,nao_v5_collision)

if __name__ == '__main__':
    main()
