# from RelaxedIK.GROOVE_RelaxedIK.relaxedIK_vars import RelaxedIK_vars
from lively_ik.groove.relaxed_ik_container import RelaxedIKContainer
import os
import yaml
import numpy.random as r
import os
import sys


class CollisionVars:
    def __init__(self, info):
        self.vars = RelaxedIKContainer(info,config_override=False, pre_config=True)


def get_score(x, CollisionVars):
    frames = CollisionVars.vars.robot.getFrames(x)
    return CollisionVars.vars.collision_graph.get_collision_score(frames)

if __name__ == "__main__":
    file = sys.argv[1]
    import rclpy
    rclpy.init()

    with open(file,'r') as data:
        info = yaml.safe_load(data)

    cv = CollisionVars(info)
    print(get_score(info["starting_config"],cv))
