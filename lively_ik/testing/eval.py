import yaml
import lively_ik
import os
from lively_ik import SRC, BASE, INFO_PARAMS, get_configs
import random
from lively_ik.groove.relaxed_ik_container import RelaxedIKContainer
import time
import json
import pandas
from wisc_actions.elements import Position, Orientation, Pose, ModeTrajectory, PoseTrajectory

BIAS_LOOKUP = {'panda':{'positive':[0.5,0.1,1.0],
                        'neutral':[0.5,0.5,0.5],
                        'negative':[1.0,0.1,0.1]},
               'ur5_robotiq_85':{'positive':[0.5,0.1,1.0],
                                 'neutral':[0.5,0.5,0.5],
                                 'negative':[1.0,0.1,0.1]},
               'nao_v5':{'positive':[0.1,0.75,1.5],
                         'neutral':[0.75,0.75,0.75],
                         'negative':[0.1,1.5,0.1]}
              }

class Test(object):
    def __init__(self,name,config,valence):
        self.name = name
        self.config = config
        self.active = False
        self.container = RelaxedIKContainer(config)
        self.valence = valence
        self.bias = [0.5,0.5,0.5]
        self.frequency = 5
        if valence == 'positive':
            self.frequency = 3
        elif valence == 'negative':
            self.frequency = 10
        self.bias = BIAS_LOOKUP[self.config['robot_name']][valence]
        self.initial_joints = config['starting_config']
        self.objectives = config['objectives']
        self.initial_weights = [obj['weight'] for obj in self.objectives]
        self.objective_types = [obj['type'] for obj in self.objectives]
        self.seeds = [random.random()*1000 for j in self.config['joint_ordering']]

    @property
    def initial_positions(self):
        return self.container.robot.get_ee_positions(self.initial_joints)

    @property
    def inital_rotations(self):
        return self.container.robot.get_ee_rotations(self.initial_joints)

    def __len__(self):
        last_update = 0
        for update in self.update_description:
            if update['time'] > last_update:
                last_update = update['time']
        return last_update

    @property
    def compiled(self):
        last_update = 0
        update_description = self.update_description
        compiled = {
            'pose':[],
            'weight':[],
            'dc':[],
            'bias':[ModeTrajectory([{'time':0,'mode':self.bias[0]}],kind=1),
                    ModeTrajectory([{'time':0,'mode':self.bias[1]}],kind=1),
                    ModeTrajectory([{'time':0,'mode':self.bias[2]}],kind=1)]
        }
        raw_pose_trajectories = []
        raw_weight_trajectories = []
        raw_dc_trajectories = []
        for pose in self.poses_from_joints(self.initial_joints):
            raw_pose_trajectories.append([{'time':0,'pose':pose}])
        for weight in self.initial_weights:
            raw_weight_trajectories.append([{'time':0,'mode':weight}])
        for dc in self.initial_joints:
            raw_dc_trajectories.append([{'time':0,'mode':dc}])
        for update in update_description:
            if update['time'] > last_update:
                last_update = update['time']
            if update['type'] == 'pose':
                raw_pose_trajectories[update['idx']].append({'time':update['time'],'pose':update['value']})
            elif update['type'] == 'weight':
                raw_weight_trajectories[update['idx']].append({'time':update['time'],'mode':update['value']})
            elif update['type'] == 'dc':
                raw_dc_trajectories[update['idx']].append({'time':update['time'],'mode':update['value']})
        for pose_trajectory in raw_pose_trajectories:
            compiled['pose'].append(PoseTrajectory(pose_trajectory,kind=1))
        for weight_trajectory in raw_weight_trajectories:
            compiled['weight'].append(ModeTrajectory(weight_trajectory,kind=1))
        for dc_trajectory in raw_dc_trajectories:
            compiled['dc'].append(ModeTrajectory(dc_trajectory,kind=1))
        return compiled

    def joint_idx(self,joint_name):
        return self.config['joint_ordering'].index(joint_name)

    def objective_joint(self,objective_idx):
        objective = self.config['objectives'][objective_idx]
        if objective['type'] in ['position','rotation','positional_noise','rotational_noise']:
            return self.config['ee_fixed_joints'][objective['index']-1]
        elif objective['type'] in ['dc','dc_noise']:
            return self.config['joint_ordering'][objective['index']-1]
        elif 'match' in objective['type']:
            return [self.config['joint_ordering'][objective['index_1']-1],self.config['joint_ordering'][objective['index_2']-1]]
        else:
            return None

    def positions_from_joints(self,joints):
        return [Position(pos[0],pos[1],pos[2]) for pos in self.container.robot.get_ee_positions(joints)]

    def rotations_from_joints(self,joints):
        return [Orientation(ori[0],ori[1],ori[2],ori[3]) for ori in self.container.robot.get_ee_rotations(joints)]

    def poses_from_joints(self,joints):
        positions = self.positions_from_joints(joints)
        rotations = self.rotations_from_joints(joints)
        poses = []
        for i in range(len(positions)):
            poses.append(Pose(positions[i],rotations[i]))
        return poses

    @property
    def initial(self):
        return self[0]

    def __getitem__(self,time):
        compiled = self.compiled
        update = {
            'pose':[],
            'dc':[],
            'bias':[],
            'weight':[]
        }
        for pose_trajectory in compiled['pose']:
            update['pose'].append(pose_trajectory[time].quaternion_dict)
        for mode_trajectory in compiled['dc']:
            update['dc'].append(mode_trajectory[time])
        for mode_trajectory in compiled['weight']:
            update['weight'].append(mode_trajectory[time])
        for mode_trajectory in compiled['bias']:
            update['bias'].append(mode_trajectory[time])
        return update

    def generate(self):
        contents = []
        # Initialize for 60 seconds
        total_time = 0
        time = 0
        while time <= 240:
            contents.append({'time':total_time,'update':self.initial,'metadata':'buffer'})
            time += 0.1
            total_time += 0.1

        time = 0
        while time <= len(self):
            contents.append({'time':total_time,'update':self[time],'metadata':'task'})
            time += 0.01
            total_time += 0.05

        with open(SRC+'/eval/'+self.name+'.json','w') as stream:
            stream.write(json.dumps(contents,indent=3))

        # Construct summary tables for weights and DC values
        weight_names = []
        for idx,objective in enumerate(self.objectives):
            if 'noise' in objective['type']:
                name = '{0}-{1} ({2})'.format(objective['type'],self.objective_joint(idx),objective['frequency'])
            elif 'position' in objective['type'] or 'rotation' in objective['type'] or 'dc' in objective['type']:
                name = '{0}-{1}'.format(objective['type'],self.objective_joint(idx))
            else:
                name = objective['type']
            weight_names.append(name)
        indices = [update['time'] for update in contents]
        weights_table = {col:[None for i in indices] for col in weight_names}
        joints_table = {col:[None for i in indices] for col in self.config['joint_ordering']}
        for cluster_idx, update_cluster in enumerate(contents):
            update = update_cluster['update']
            for idx,weight in enumerate(update['weight']):
                weights_table[weight_names[idx]][cluster_idx] = weight
            for idx,joint in enumerate(update['dc']):
                joints_table[self.config['joint_ordering'][idx]][cluster_idx] = joint
        weights_df = pandas.DataFrame(weights_table)
        joints_df = pandas.DataFrame(joints_table)
        weights_df.to_csv(SRC+'/eval/'+self.name+'_weights_summary.csv')
        joints_df.to_csv(SRC+'/eval/'+self.name+'_joints_summary.csv')

    @property
    def update_description(self):
        commands = []
        for idx,pose in enumerate(self.poses_from_joints(self.initial_joints)):
            update = {'type':'pose','value':pose,'idx':idx,'time':30}
            commands.append(update)
        for idx,weight in enumerate(self.initial_weights):
            update = {'type':'weight','value':weight,'idx':idx,'time':30}
            commands.append(update)
        for idx,bias in enumerate(self.bias):
            update = {'type':'bias','value':bias,'idx':idx,'time':30}
            commands.append(update)
        for idx,dc in enumerate(self.initial_joints):
            update = {'type':'dc','value':dc,'idx':idx,'time':30}
            commands.append(update)
        return commands

class NaoStaticValence(Test):
    def __init__(self,valence):
        config = get_configs()['nao_v5']
        super(NaoStaticValence,self).__init__('nao_static_weights_{0}'.format(valence),config,valence)
        if valence == 'positive':
            self.initial_joints[self.joint_idx('HeadPitch')] = -0.15
            self.initial_joints[self.joint_idx('LShoulderPitch')] = 1.6
            self.initial_joints[self.joint_idx('LShoulderRoll')] = 0.45
            self.initial_joints[self.joint_idx('RShoulderPitch')] = 1.6
            self.initial_joints[self.joint_idx('RShoulderRoll')] = -0.45
        elif valence == 'negative':
            self.initial_joints[self.joint_idx('HeadPitch')] = 0.2
            self.initial_joints[self.joint_idx('LElbowRoll')] = -0.1
            self.initial_joints[self.joint_idx('RElbowRoll')] = 0.1
        for idx,objective in enumerate(self.config['objectives']):
            if 'noise' in objective['type'] and objective['frequency'] == self.frequency:
                if objective['type'] == 'positional_noise':
                    # Make the weight 20
                    self.initial_weights[idx] = 20
                elif objective['type'] == 'rotational_noise':
                    # Make the weight 5
                    self.initial_weights[idx] = 5
                elif objective['type'] == 'dc_noise' and self.objective_joint(idx) in ['LHand','RHand']:
                    # Make the weight 20
                    self.initial_weights[idx] = 20
            elif 'noise' in objective['type']:
                # Make the weight 0
                self.initial_weights[idx] = 0

    @property
    def update_description(self):
        commands = []
        for idx,pose in enumerate(self.poses_from_joints(self.initial_joints)):
            update = {'type':'pose','value':pose,'idx':idx,'time':30}
            commands.append(update)
        for idx,weight in enumerate(self.initial_weights):
            update = {'type':'weight','value':weight,'idx':idx,'time':30}
            commands.append(update)
        for idx,bias in enumerate(self.bias):
            update = {'type':'bias','value':bias,'idx':idx,'time':30}
            commands.append(update)
        for idx,dc in enumerate(self.initial_joints):
            update = {'type':'dc','value':dc,'idx':idx,'time':30}
            commands.append(update)
        return commands

class NaoWaveAction(Test):
    def __init__(self):
        config = get_configs()['nao_v5']
        super(NaoWaveAction,self).__init__('nao_wave_task',config,'positive')
        self.initial_joints[self.joint_idx('HeadPitch')] = -0.15
        self.initial_joints[self.joint_idx('LShoulderPitch')] = 1.6
        self.initial_joints[self.joint_idx('LShoulderRoll')] = 0.45
        self.initial_joints[self.joint_idx('RShoulderPitch')] = 1.6
        self.initial_joints[self.joint_idx('RShoulderRoll')] = -0.45

        # Drop all noise weights for everything except the desired frequency.
        # Only allow dc noise in the R/L Hands
        for idx,objective in enumerate(self.config['objectives']):
            if 'noise' in objective['type'] and objective['frequency'] == self.frequency:
                if objective['type'] == 'positional_noise':
                    # Make the weight 20
                    self.initial_weights[idx] = 20
                elif objective['type'] == 'rotational_noise':
                    # Make the weight 5
                    self.initial_weights[idx] = 5
                elif objective['type'] == 'dc_noise' and self.objective_joint(idx) in ['LHand','RHand']:
                    # Make the weight 20
                    self.initial_weights[idx] = 20
            elif 'noise' in objective['type']:
                # Make the weight 0
                self.initial_weights[idx] = 0

    @property
    def update_description(self):
        commands = []

        for idx,objective in enumerate(self.config['objectives']):
            ### Reduce weight on right arm position/orientation objectives
            if self.objective_joint(idx) == 'RArm_effector_fixedjoint':
                # print("Setting RArm {0} objective to 0".format(objective['type']))
                update = {'type':'weight','value':0,'idx':idx,'time':5}
                commands.append(update)
            ### Increase the weight on joint objectives for the right arm
            elif self.objective_joint(idx) in ['RShoulderPitch','RShoulderRoll','RElbowYaw','RElbowRoll','RWristYaw']:
                if objective['type'] == 'dc_noise' and objective['frequency'] == self.frequency:
                    # print("Setting DC noise weight {0} ({1}) objective to 5".format(self.objective_joint(idx),objective['frequency']))
                    update = {'type':'weight','value':5,'idx':idx,'time':5}
                    commands.append(update)
                elif objective['type'] == 'dc':
                    # print("Setting DC weight {0} objective to 15".format(self.objective_joint(idx)))
                    update = {'type':'weight','value':15,'idx':idx,'time':5}
                    commands.append(update)
                elif objective['type'] == 'dc_noise' and objective['frequency'] != self.frequency:
                    # print("Setting DC noise weight {0} ({1}) objective to 0".format(self.objective_joint(idx),objective['frequency']))
                    update = {'type':'weight','value':0,'idx':idx,'time':5}
                    commands.append(update)

        wave = [{'RShoulderPitch':-1.3,'RShoulderRoll':1.35,},
                {'RWristYaw':-0.4,'RShoulderRoll':0.1,'RElbowRoll':0.35,'RElbowYaw':0.0},
                {'RShoulderRoll':-0.6,'RElbowRoll':1.35,'RElbowYaw':0.0},
                {'RShoulderRoll':0.1,'RElbowRoll':0.35,'RElbowYaw':0.0},
                {'RShoulderPitch':-1.3,'RWristYaw':-0.4,'RShoulderRoll':-0.6,'RElbowRoll':1.35,'RElbowYaw':0.0},
                {'RShoulderPitch':1.6,'RWristYaw':1.3,'RShoulderRoll':-0.45,'RElbowRoll':0.5,'RElbowYaw':0.5}
        ]


        joints = [v for v in self.initial_joints]
        time = 6
        for step in wave:

            # DC Values
            for joint_name, dc in step.items():
                idx = self.config['joint_ordering'].index(joint_name)
                joints[idx] = dc
                commands.append({'type':'dc','value':dc,'idx':idx,'time':time})
            # poses = self.poses_from_joints(joints)
            # for idx,ee in enumerate(self.config['ee_fixed_joints']):
            #     commands.append({'type':'pose','value':poses[idx],'idx':idx,'time':time})

            # Objective weights
            for idx,objective in enumerate(self.config['objectives']):
                ### Reduce weight on right arm position/orientation objectives
                if self.objective_joint(idx) == 'RArm_effector_fixedjoint':
                    update = {'type':'weight','value':0,'idx':idx,'time':time}
                    commands.append(update)
                ### Increase the weight on joint objectives for the right arm
                elif self.objective_joint(idx) in ['RShoulderPitch','RShoulderRoll','RElbowYaw','RElbowRoll','RWristYaw']:
                    if objective['type'] == 'dc_noise' and objective['frequency'] == self.frequency:
                        update = {'type':'weight','value':5,'idx':idx,'time':time}
                        commands.append(update)
                    elif objective['type'] == 'dc':
                        update = {'type':'weight','value':15,'idx':idx,'time':time}
                        commands.append(update)
                    elif objective['type'] == 'dc_noise' and objective['frequency'] != self.frequency:
                        update = {'type':'weight','value':0,'idx':idx,'time':time}
                        commands.append(update)
            time += 0.5

        for time in [15,16,17,18,19,20]:
            for idx,pose in enumerate(self.poses_from_joints(self.initial_joints)):
                update = {'type':'pose','value':pose,'idx':idx,'time':time}
                commands.append(update)
            for idx,weight in enumerate(self.initial_weights):
                update = {'type':'weight','value':weight,'idx':idx,'time':time}
                commands.append(update)
            for idx,bias in enumerate(self.bias):
                update = {'type':'bias','value':bias,'idx':idx,'time':time}
                commands.append(update)
            for idx,dc in enumerate(self.initial_joints):
                update = {'type':'dc','value':dc,'idx':idx,'time':time}
                commands.append(update)

        return commands

class PandaStaticValence(Test):
    def __init__(self,valence):
        config = get_configs()['panda']
        super(PandaStaticValence,self).__init__('panda_static_weights_{0}'.format(valence),config,valence)
        if valence == 'positive':
            self.initial_joints[self.joint_idx('panda_joint2')] = -0.1
        elif valence == 'negative':
            self.initial_joints[self.joint_idx('panda_joint2')] = 0.55
        for idx,objective in enumerate(self.config['objectives']):
            if 'noise' in objective['type'] and objective['frequency'] == self.frequency:
                if objective['type'] == 'positional_noise':
                    # Make the weight 20
                    self.initial_weights[idx] = 20
                elif objective['type'] == 'rotational_noise':
                    # Make the weight 5
                    self.initial_weights[idx] = 5
                elif objective['type'] == 'dc_noise' and self.objective_joint(idx) in ['panda_finger_joint1','panda_finger_joint2']:
                    # Make the weight 20
                    self.initial_weights[idx] = 20
            elif 'noise' in objective['type']:
                # Make the weight 0
                self.initial_weights[idx] = 0

    @property
    def update_description(self):
        commands = []
        for idx,pose in enumerate(self.poses_from_joints(self.initial_joints)):
            update = {'type':'pose','value':pose,'idx':idx,'time':30}
            commands.append(update)
        for idx,weight in enumerate(self.initial_weights):
            update = {'type':'weight','value':weight,'idx':idx,'time':30}
            commands.append(update)
        for idx,bias in enumerate(self.bias):
            update = {'type':'bias','value':bias,'idx':idx,'time':30}
            commands.append(update)
        for idx,dc in enumerate(self.initial_joints):
            update = {'type':'dc','value':dc,'idx':idx,'time':30}
            commands.append(update)
        return commands

class UR5PickupAction(Test):
    def __init__(self):
        config = get_configs()['ur5_robotiq_85']
        super(UR5PickupAction,self).__init__('ur5_pickup_task',config,'neutral')
        for idx,objective in enumerate(self.config['objectives']):
            if 'noise' in objective['type'] and objective['frequency'] == self.frequency:
                if objective['type'] == 'positional_noise':
                    # Make the weight 20
                    self.initial_weights[idx] = 20
                elif objective['type'] == 'rotational_noise':
                    # Make the weight 5
                    self.initial_weights[idx] = 5
                elif objective['type'] == 'dc_noise' and self.objective_joint(idx) in ['gripper_finger1_joint','gripper_finger2_joint']:
                    # Make the weight 20
                    self.initial_weights[idx] = 20
            elif 'noise' in objective['type']:
                # Make the weight 0
                self.initial_weights[idx] = 0

    @property
    def update_description(self):
        commands = []
        for idx,pose in enumerate(self.poses_from_joints(self.initial_joints)):
            update = {'type':'pose','value':pose,'idx':idx,'time':30}
            commands.append(update)
        for idx,weight in enumerate(self.initial_weights):
            update = {'type':'weight','value':weight,'idx':idx,'time':30}
            commands.append(update)
        for idx,bias in enumerate(self.bias):
            update = {'type':'bias','value':bias,'idx':idx,'time':30}
            commands.append(update)
        for idx,dc in enumerate(self.initial_joints):
            update = {'type':'dc','value':dc,'idx':idx,'time':30}
            commands.append(update)
        return commands

def clamp(x, lo, hi):
    if lo <= x <= hi:
        return x
    elif x < lo:
        return lo
    elif x > hi:
        return hi

def main():
    # NaoStaticValence('positive').generate()
    # NaoStaticValence('negative').generate()
    action = NaoWaveAction()
    # node.execute_panda()
    # node.execute_ur5()

if __name__ == '__main__':
    main()
