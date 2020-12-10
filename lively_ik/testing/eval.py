import yaml
import lively_ik
import os
from lively_ik import SRC, BASE, INFO_PARAMS, get_configs
import random
from lively_ik.groove.relaxed_ik_container import RelaxedIKContainer
import time
import json
import pandas
from wisc_actions.elements import Position, Orientation, Pose
from scipy import interpolate

BIAS_LOOKUP = {'panda':{'positive':[0.1,0.75,1.5],
                        'neutral':[0.5,0.5,0.5],
                        'negative':[0.1,1.5,0.1]},
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
        self.joint_limits = config['joint_limits']

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
        update_description = self.update_description
        compiled = {
            'weight':[],
            'dc':[],
            'bias':[[{'time':0,'mode':self.bias[0]},{'time':1,'mode':self.bias[0]}],
                    [{'time':0,'mode':self.bias[1]},{'time':1,'mode':self.bias[1]}],
                    [{'time':0,'mode':self.bias[2]},{'time':1,'mode':self.bias[2]}]]
        }
        for weight in self.initial_weights:
            compiled['weight'].append([{'time':0,'mode':weight}])
        for dc in self.initial_joints:
            compiled['dc'].append([{'time':0,'mode':dc}])
        for update in update_description:
            if update['type'] == 'weight':
                compiled['weight'][update['idx']].append({'time':update['time'],'mode':update['value']})
            elif update['type'] == 'dc':
                compiled['dc'][update['idx']].append({'time':update['time'],'mode':update['value']})
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

    @property
    def interp(self):
        trajectories = {'weight':[],
                        'dc':[],
                        'bias':[]}
        compiled = self.compiled
        for field in ['weight','dc','bias']:
            for trajectory in compiled[field]:
                times = [float(wp['time']) for wp in trajectory]
                values = [float(wp['mode']) for wp in trajectory]
                if len(times) == 1:
                    times.append(times[0]+1)
                    values.append(values[0])
                trajectories[field].append(interpolate.interp1d(times,values,kind='slinear',fill_value='extrapolate'))
        return trajectories

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
        interp = self.interp
        update = {
            'pose':[],
            'dc':[],
            'bias':[],
            'weight':[]
        }
        joints = [0]*len(self.initial_joints)
        for field in ['dc','weight','bias']:
            for i,fn in enumerate(interp[field]):
                value = float(fn(time))
                if field == 'dc':
                    joints[i] = value
                update[field].append(value)
        update['pose'] = [pose.quaternion_dict for pose in self.poses_from_joints(joints)]
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
                    self.initial_weights[idx] = 5
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
                    self.initial_weights[idx] = 5
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
            self.initial_joints[self.joint_idx('panda_joint2')] = 0.35
        for idx,objective in enumerate(self.config['objectives']):
            if 'noise' in objective['type'] and objective['frequency'] == self.frequency:
                if objective['type'] == 'positional_noise':
                    # Make the weight 20
                    self.initial_weights[idx] = 25
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

class PandaPickupAction(Test):
    def __init__(self):
        config = get_configs()['panda']
        super(PandaPickupAction,self).__init__('panda_pickup_task',config,'neutral')
        for idx,objective in enumerate(self.config['objectives']):
            if 'noise' in objective['type'] and objective['frequency'] == self.frequency:
                if objective['type'] == 'positional_noise':
                    # Make the weight 20
                    self.initial_weights[idx] = 25
                elif objective['type'] == 'rotational_noise':
                    # Make the weight 5
                    self.initial_weights[idx] = 5
                elif objective['type'] == 'dc_noise' and self.objective_joint(idx) in ['panda_finger_joint1','panda_finger_joint2']:
                    # Make the weight 20
                    self.initial_weights[idx] = 10
            elif 'noise' in objective['type']:
                # Make the weight 0
                self.initial_weights[idx] = 0

    @property
    def update_description(self):

        task = [
                # Before beginning action
                {'joints':{'panda_joint1':0,
                           'panda_joint2':0,
                           'panda_joint3':0,
                           'panda_joint4':-1.52715,
                           'panda_joint5':0,
                           'panda_joint6':1.8675,
                           'panda_joint7':0.8,
                           'panda_finger_joint1':0.02,
                           'panda_finger_joint2':0.02},
                 'noise':True,
                 'time':3.0},
                {'joints':{},
                  'noise':True,
                  'time':4.0},
                # Move to duck approach, open gripper
                {'joints':{'panda_joint1':0.33,
                           'panda_joint2':0.25,
                           'panda_joint3':0.0,
                           'panda_joint4':-2.35,
                           'panda_joint5':0.0,
                           'panda_joint6':2.68,
                           'panda_joint7':1.13,
                           'panda_finger_joint1':0.025,
                           'panda_finger_joint2':0.025},
                 'noise':False,
                 'time':6.0},
                # Move to grasp duck
                {'joints':{'panda_joint1':0.33,
                           'panda_joint2':0.45,
                           'panda_joint3':0.0,
                           'panda_joint4':-2.26,
                           'panda_joint5':0.0,
                           'panda_joint6':2.8,
                           'panda_joint7':1.13,
                           'panda_finger_joint1':0.020,
                           'panda_finger_joint2':0.020},
                 'noise':False,
                 'time':6.5},
                # Grasp duck
                {'joints':{'panda_joint1':0.35,
                           'panda_joint2':0.43,
                           'panda_joint3':0.0,
                           'panda_joint4':-2.26,
                           'panda_joint5':0.0,
                           'panda_joint6':2.8,
                           'panda_joint7':1.13,
                           'panda_finger_joint1':0.016,
                           'panda_finger_joint2':0.016},
                 'noise':False,
                 'time':7.5},
                # Retract
                {'joints':{'panda_joint1':0.33,
                           'panda_joint2':0.25,
                           'panda_joint3':0.0,
                           'panda_joint4':-2.35,
                           'panda_joint5':0.0,
                           'panda_joint6':2.68,
                           'panda_joint7':1.13},
                 'noise':False,
                 'time':8.0},
                # Move to box approach
                {'joints':{'panda_joint1':-0.46,
                           'panda_joint2':-0.21,
                           'panda_joint3':0.0,
                           'panda_joint4':-2.25,
                           'panda_joint5':0.0,
                           'panda_joint6':2.14,
                           'panda_joint7':0.3,
                           'panda_finger_joint1':0.016,
                           'panda_finger_joint2':0.016},
                 'noise':False,
                 'time':9.0},
                # Release
                {'joints':{'panda_finger_joint1':0.02,
                           'panda_finger_joint2':0.02},
                 'noise':False,
                 'time':9.25},
                # Hold
                {'joints':{'panda_joint1':-0.46,
                           'panda_joint2':-0.21,
                           'panda_joint3':0.0,
                           'panda_joint4':-2.25,
                           'panda_joint5':0.0,
                           'panda_joint6':2.14,
                           'panda_joint7':0.3,
                           'panda_finger_joint1':0.02,
                           'panda_finger_joint2':0.02},
                 'noise':False,
                 'time':11},
                # Resume Liveliness
                {'joints':{'panda_joint1':0,
                           'panda_joint2':0,
                           'panda_joint3':0,
                           'panda_joint4':-1.52715,
                           'panda_joint5':0,
                           'panda_joint6':1.8675,
                           'panda_joint7':0.8,
                           'panda_finger_joint1':0.02,
                           'panda_finger_joint2':0.02},
                 'noise':False,
                 'time':12.0},
                # Terminate
                {'joints':{'panda_joint1':0,
                           'panda_joint2':0,
                           'panda_joint3':0,
                           'panda_joint4':-1.52715,
                           'panda_joint5':0,
                           'panda_joint6':1.8675,
                           'panda_joint7':0.8},
                 'noise':True,
                 'time':15},
        ]

        joints = [v for v in self.initial_joints]
        commands = []

        for step in task:

            # DC Values
            for joint_name, dc in step['joints'].items():
                idx = self.config['joint_ordering'].index(joint_name)
                joints[idx] = dc
                commands.append({'type':'dc','value':dc,'idx':idx,'time':step['time']})
            poses = self.poses_from_joints(joints)
            for idx,ee in enumerate(self.config['ee_fixed_joints']):
                commands.append({'type':'pose','value':poses[idx],'idx':idx,'time':step['time']})

            # Objective weights
            positive_noise_weights = {'positional_noise':20,'rotational_noise':5,'dc_noise':10}

            for idx,objective in enumerate(self.config['objectives']):
                ## Set noise objectives
                if step['noise']:
                    ### Set the noise objective to positive if the right frequency
                    if 'noise' in objective['type'] and objective['frequency'] == self.frequency:
                        update = {'type':'weight','value':positive_noise_weights[objective['type']],'idx':idx,'time':step['time']}
                        commands.append(update)
                    elif 'noise' in objective['type'] and objective['frequency'] != self.frequency:
                        update = {'type':'weight','value':0,'idx':idx,'time':step['time']}
                        commands.append(update)
                elif 'noise' in objective['type']:
                    update = {'type':'weight','value':0,'idx':idx,'time':step['time']}
                    commands.append(update)


        return commands

class PandaProfiler(Test):
    def __init__(self,num_poses=25):
        config = get_configs()['panda']
        self.num_poses = num_poses
        super(PandaProfiler,self).__init__('panda_profiler',config,'neutral')
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
                    self.initial_weights[idx] = 10
            elif 'noise' in objective['type']:
                # Make the weight 0
                self.initial_weights[idx] = 0

    @property
    def update_description(self):

        task = [{'joints':{joint:self.initial_joints[j] for j,joint in enumerate(self.config['joint_ordering'])},
                 'noise':True,
                 'time':0}]

        for i in range(1,self.num_poses+1):
            joint_set = {joint:random.uniform(self.joint_limits[j][0],self.joint_limits[j][1]) for j,joint in enumerate(self.config['joint_ordering'])}
            for offset in [-20,0]:
                step = {'joints':joint_set,
                        'noise':True,
                        'time':i*80+offset
                }
                task.append(step)

        joints = [v for v in self.initial_joints]
        commands = []

        for step in task:
            # DC Values
            for joint_name, dc in step['joints'].items():
                idx = self.config['joint_ordering'].index(joint_name)
                commands.append({'type':'dc','value':dc,'idx':idx,'time':step['time']})

        return commands

class NaoProfiler(Test):
    def __init__(self,num_poses=25):
        self.num_poses = num_poses
        config = get_configs()['nao_v5']
        super(NaoProfiler,self).__init__('nao_v5_profiler',config,'neutral')
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
                    self.initial_weights[idx] = 10
            elif 'noise' in objective['type']:
                # Make the weight 0
                self.initial_weights[idx] = 0

    @property
    def update_description(self):

        task = [{'joints':{joint:self.initial_joints[j] for j,joint in enumerate(self.config['joint_ordering'])},
                'noise':True,
                'time':0}]

        for i in range(1,self.num_poses+1):
            joint_set = {joint:random.uniform(self.joint_limits[j][0],self.joint_limits[j][1]) for j,joint in enumerate(self.config['joint_ordering'])}
            for offset in [-20,0]:
                step = {'joints':joint_set,
                        'noise':True,
                        'time':i*80+offset
                }
                task.append(step)

        joints = [v for v in self.initial_joints]
        commands = []

        for step in task:
            # DC Values
            for joint_name, dc in step['joints'].items():
                idx = self.config['joint_ordering'].index(joint_name)
                joints[idx] = dc
                commands.append({'type':'dc','value':dc,'idx':idx,'time':step['time']})
            poses = self.poses_from_joints(joints)
            for idx,ee in enumerate(self.config['ee_fixed_joints']):
                commands.append({'type':'pose','value':poses[idx],'idx':idx,'time':step['time']})

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
