import yaml
import lively_ik
import rclpy
from rclpy.node import Node
import os
from lively_ik import SRC, BASE, INFO_PARAMS, get_configs
from lively_ik.testing.tests import NaoStaticValence, NaoWaveAction, PandaStaticValence, UR5PickupAction
from julia import LivelyIK
import random
from lively_ik.groove.relaxed_ik_container import RelaxedIKContainer
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

def main():

    tests = [
        NaoStaticValence('positive'),
        NaoStaticValence('negative'),
        NaoWaveAction(),
        PandaStaticValence('positive'),
        PandaStaticValence('negative')
    ]
    for test in tests:
        test.generate()
    del tests

    configs = get_configs()
    files = {
    'nao_static_weights_negative':{'frequency':10,'config':configs['nao_v5']},
    'nao_static_weights_positive':{'frequency':3,'config':configs['nao_v5']},
    'nao_wave_task':{'frequency':3,'config':configs['nao_v5']},
    'panda_static_weights_negative':{'frequency':10,'config':configs['panda']},
    'panda_static_weights_positive':{'frequency':3,'config':configs['panda']}
    }
    for file in files:
        fn = 'julia {0}/lively_ik/EvalNode.jl {0}/config/info_files/{1}.yaml {0}/eval/{2}.json'.format(SRC,files[file]['config']['robot_name'],file)
        subprocess.run(fn,shell=True)

    for file in files:
        with open(SRC+'/eval/'+file+'_sol.json','r') as stream:
            data = json.load(stream)
        allowed_joints = ALLOWED_NAIVE_JOINTS[files[file]['config']['robot_name']]
        joint_ordering = files[file]['config']['joint_ordering']
        df_data = {col:[] for col in ['condition']+joint_ordering}
        seeds = [random.random()*1000 for j in joint_ordering]
        joint_range = {joint:0 for joint in allowed_joints}
        for joint in allowed_joints:
            lively = values_for_joint(files[file]['config'],joint,data,'lik_sol')
            static = values_for_joint(files[file]['config'],joint,data,'rik_sol')
            dc = values_for_joint(files[file]['config'],joint,data,'dc')
            diff = [lively[i]-static[i] for i in range(len(lively))]
            joint_range[joint] = np.std(diff)*3
        for step in data:
            time = step['time']
            df_data['condition'].extend(['lively','static','naive','dc'])
            for idx,joint in enumerate(joint_ordering):
                df_data[joint].append(step['lik_sol'][idx])
                df_data[joint].append(step['rik_sol'][idx])
                df_data[joint].append(step['dc'][idx])
                if joint in allowed_joints:
                    value = step['dc'][idx] + LivelyIK.noise1D(time,seeds[idx],files[file]['frequency']) * joint_range[joint]
                    value = clamp(value,files[file]['config']['joint_limits'][idx][0],files[file]['config']['joint_limits'][idx][1])
                    df_data[joint].append(value)
                else:
                    df_data[joint].append(step['rik_sol'][idx])
        df = pandas.DataFrame(df_data)
        df.to_csv(SRC+'/eval/'+file+'.csv')

if __name__ == '__main__':
    main()
