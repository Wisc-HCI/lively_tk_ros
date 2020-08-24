from ..groove.relaxed_ik_container import RelaxedIKContainer
import os
import yaml
import numpy.random as r
import os
import sys


class CollisionVars:
    def __init__(self, info, rcl_node):
        self.vars = RelaxedIKContainer(info,rcl_node,config_override=False, pre_config=True)


def get_score(x, CollisionVars):
    frames = CollisionVars.vars.robot.getFrames(x)
    return CollisionVars.vars.collision_graph.get_collision_score(frames)

if __name__ == "__main__":
    file = sys.argv[1]
    import rclpy
    from rclpy.node import Node
    rclpy.init()
    node = Node('lively_ik')

    with open(file,'r') as data:
        info = yaml.safe_load(data)

    cv = CollisionVars(info, node)
    print(get_score(info["starting_config"],cv))
