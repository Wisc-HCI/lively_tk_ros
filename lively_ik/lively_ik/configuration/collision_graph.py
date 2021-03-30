from lively_ik.configuration.collision_container import CollisionObjectContainer
import numpy as np
import itertools
import math


class CollisionGraph:
    def __init__(self, config, robot, link_exclusion_list = [], sample_states=[]):
        self.c = CollisionObjectContainer(config)
        self.c.add_collision_objects_from_robot(robot, link_exclusion_list)
        self.robot = robot
        self.sample_states = sample_states
        self.num_objects = len(self.c.collision_objects)
        # self.b_value = 1.0/self.num_objects
        self.b_value = 5.0
        self.original_distances = 10000*np.ones((self.num_objects, self.num_objects))
        self.combinations = list(itertools.combinations(range(self.num_objects),r=2))
        self.collision_color_array = self.num_objects*[0]
        self.danger_dis = 0.1

        self.initialize_table()
        # print('cg 2.5')
        self.c_values = self.get_c_values(self.original_distances)

    def get_collision_score_of_state(self, transform, state):
        frames = self.robot.getFrames(transform, state)
        return self.get_collision_score(transform, frames)

    def get_collision_score(self, transform, frames):
        sum = 0.0
        self.c.update_all_transforms(transform, frames)
        c_values = self.c_values
        for i, pair in enumerate(self.combinations):
            l1 = pair[0]
            l2 = pair[1]

            dis = self.c.get_min_distance(l1,l2)
            if dis == -1:
                dis = 0.0
            # print dis

            c = c_values[l1,l2]
            if not c == 0.0:
                sum += self.b_value * (math.e ** ((-(dis) ** 4.0) / (2.0 * c ** 2)))

        return sum

    def initialize_table(self):
        for (t,s) in self.sample_states:
            frames = self.robot.getFrames(t,s)
            self.c.update_all_transforms(frames)
            for pair in self.combinations:
                a = pair[0]
                b = pair[1]
                dis = self.c.get_min_distance(a,b)
                if dis == -1:
                    dis = 0.0
                if dis < self.original_distances[a,b]:
                    self.original_distances[a, b] = dis
                    self.original_distances[b, a] = dis

            for pair in self.combinations:
                a = pair[0]
                b = pair[1]
                if not self.original_distances[a,b] == 0.0:
                    self.original_distances[a,b] = min( self.danger_dis, self.original_distances[a,b] )
                if not self.original_distances[b,a] == 0.0:
                    self.original_distances[b,a] = min( self.danger_dis, self.original_distances[b,a] )


    def get_c_values(self, original_distances):
        shape = self.original_distances.shape
        c_values = np.zeros(shape)
        for i in range(shape[0]):
            for j in range(shape[1]):
                if original_distances[i,j] <= .001:
                    c_values[i,j] = 0.0
                else:
                    c_values[i,j] = self.get_c_value_from_dis(original_distances[i,j], self.b_value)
        return c_values

    def get_c_value_from_dis(self, dis, b, v=1.0e-15):
        return math.sqrt(-(dis) ** 4 / (2.0 * math.log(v / b)))
