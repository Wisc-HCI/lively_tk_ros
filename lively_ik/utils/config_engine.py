import numpy as np
from .colors import bcolors
from os import listdir
# from sklearn.externals import joblib
import joblib
import bson
from .neural_net_trainer import CollisionNNTrainer
import os
from lively_ik import BASE


class ConfigEngine:
    def __init__(self, info, collision_graph, vars, override=False):
        self.info = info
        self.collision_graph = collision_graph
        self.nn_file_name = BASE+'/config/collision_nn/'+self.info['robot_name']+'_python'
        self.vars = vars

        if not os.path.exists(self.nn_file_name) or override:
            print(bcolors.OKBLUE + 'Config file not found at ' +self.nn_file_name+ ', generating a new one!  This will take some time.' + bcolors.ENDC)
            self.robot_name, self.collision_nn, self.init_state, self.full_joint_lists, self.fixed_ee_joints, \
               self.joint_order, self.collision_file = self.generate_nn_file()
        else:
            try:
                self.collision_nn = joblib.load(self.nn_file_name)
            except:
                print(bcolors.OKBLUE + 'Config file at ' + self.nn_file_name + ' could not be opened; generating a new one!  This will take some time.' + bcolors.ENDC)
                self.robot_name, self.collision_nn, self.init_state, self.full_joint_lists, self.fixed_ee_joints, \
                   self.joint_order, self.collision_file = self.generate_nn_file()

    def generate_nn_file(self):
        trainer = CollisionNNTrainer(self.collision_graph)
        collision_nn = trainer.clf
        # robot_name = trainer.robot.__name__
        robot_name = 'robot'

        joblib.dump(collision_nn, self.nn_file_name)

        return robot_name, collision_nn, self.info['starting_config'], self.info['joint_names'], self.info['ee_fixed_joints'], \
               self.info['joint_ordering'], self.info['robot_name'] + '_python'
