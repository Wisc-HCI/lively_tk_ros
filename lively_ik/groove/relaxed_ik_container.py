from ..spacetime.robot import Robot
from ..utils.urdf_load import *
from ..utils.collision_graph import CollisionGraph
from ..utils.config_engine import ConfigEngine
import lively_ik
import os
import yaml
#from sklearn.externals import joblib
import joblib
import xml.etree.ElementTree as et

CONFIG_DIR = lively_ik.BASE + '/config'

class RelaxedIKContainer(object):
    def __init__(self,
                 info,
                 collision_link_exclusion_list=[],
                 config_override=False,
                 pre_config=False
                 ):
        self.info = info
        self.full_joint_lists = info['joint_names']
        self.fixed_ee_joints = info['ee_fixed_joints']
        self.joint_order = info['joint_ordering']
        self.urdf = info['urdf']
        self.c_boost = False
        self.num_chains = len(self.full_joint_lists)
        self.arms = []
        self.urdf_robots = []
        self.trees = []

        try:
            from boost import objectives_ext
            self.c_boost = True
        except:
            pass

        for i in range(self.num_chains):
            urdf_robot, arm, arm_c, tree = urdf_load_from_string(self.urdf, '', '', self.full_joint_lists[i], self.fixed_ee_joints[i])
            if self.c_boost:
                self.arms.append(arm_c)
            else:
                self.arms.append(arm)
            self.urdf_robots.append(urdf_robot)
            self.trees.append(tree)

        # make robot
        self.robot = Robot(self.arms, self.full_joint_lists, self.joint_order,extra_joints=self.get_extra_joints())

        self.numDOF = self.robot.numDOF

        self.bounds = self.robot.bounds

        self.collision_graph = CollisionGraph(self.info, self.robot, collision_link_exclusion_list)

        if not self.numDOF == len(info['starting_config']):
            print(bcolors.WARNING + 'WARNING: Length of init_state does not match number of robot DOFs.  Is this what you intended?' + bcolors.ENDC)

        if not pre_config:
            self.ce = ConfigEngine(self.info, self.collision_graph, self, override=config_override)
            self.collision_nn = self.ce.collision_nn

    def get_extra_joints(self):
        extra_joint_names = []
        for joint in self.info['joint_ordering']:
            found = False
            for chain in self.info['joint_names']:
                if joint in chain:
                    found = True
            if not found:
                extra_joint_names.append(joint)
        extra_joints = {name:{'bounds':[0,0],'velocity':0} for name in extra_joint_names}
        robot_root = et.fromstring(self.info['urdf'])
        for child in robot_root:
            if child.tag == 'joint' and child.attrib['type'] != 'fixed' and child.attrib['name'] in extra_joint_names:
                for attrib in child:
                    if attrib.tag == 'limit':
                        extra_joints[child.attrib['name']]['bounds'][0] = float(attrib.attrib['lower'])
                        extra_joints[child.attrib['name']]['bounds'][1] = float(attrib.attrib['upper'])
                        extra_joints[child.attrib['name']]['velocity'] = float(attrib.attrib['velocity'])
        return extra_joints
