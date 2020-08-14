import fcl
import numpy as np
from .colors import bcolors as bc
from visualization_msgs.msg import Marker
from .transformations import quaternion_from_euler, quaternion_from_matrix, quaternion_multiply
import rclpy
from rclpy.node import Node


class CollisionObjectContainer(object):
    def __init__(self, info, rcl_node):
        self.rcl_node = rcl_node
        self.collision_object_publisher = self.rcl_node.create_publisher(Marker,'visualization_marker',10)
        self.collision_objects = []

        keys = info.keys()
        for k in keys:
            if not info[k] == None:
                if k == 'robot_link_radius' or k == 'sample_states' or k == 'training_states' or k == 'problem_states': continue
                for i in range(len(info[k])):
                    if k == 'boxes':
                        self.collision_objects.append(Collision_Box(info[k][i]))
                    elif k == 'spheres':
                        self.collision_objects.append(Collision_Sphere(info[k][i]))
                    elif k == 'ellipsoids':
                        self.collision_objects.append(Collision_Ellipsoid(info[k][i]))
                    elif k == 'capsules':
                        self.collision_objects.append(Collision_Capsule(info[k][i]))
                    elif k == 'cones':
                        self.collision_objects.append(Collision_Cone(info[k][i]))
                    elif k == 'cylinders':
                        self.collision_objects.append(Collision_Cylinder(info[k][i]))
                    elif k == 'meshes':
                        self.collision_objects.append(Collision_Mesh(info[k][i]))

                    self.collision_objects[-1].type = 'environment_object'

        self.robot_link_radius = 0.05
        if 'robot_link_radius' in keys:
            self.robot_link_radius = float(info['robot_link_radius'])

        if 'sample_states' in keys:
            self.sample_states = info['sample_states']
        else:
            raise Exception('Must specify at least one sample state in collision yaml file!')

        self.set_rviz_ids()

    def set_rviz_ids(self):
        for i,c in enumerate(self.collision_objects):
            c.marker.id = i

    def get_min_distance(self, a, b):
        obja = self.collision_objects[a].obj
        objb = self.collision_objects[b].obj

        self.request = fcl.DistanceRequest()
        self.result = fcl.DistanceResult()

        ret = fcl.distance(obja, objb, self.request, self.result)
        return self.result.min_distance

    def add_collision_objects_from_robot(self, robot, exclusion=[]):
        numDOF = robot.numDOF

        frames_list = robot.getFrames(numDOF*[0])
        for arm_idx in range(len(frames_list)):
            frames = frames_list[arm_idx]
            jtPts = frames[0]

            numLinks = (len(jtPts)-1)
            for l in range(numLinks):
                curr_idx = numLinks*arm_idx + l
                if not curr_idx in exclusion:
                    ptA = jtPts[l]
                    ptB = jtPts[l+1]
                    midPt = ptA + 0.5*(ptB - ptA)
                    dis = np.linalg.norm(ptA - ptB)
                    if dis < 0.02:
                        continue

                    cylinder = Collision_Cylinder.init_with_arguments('robotLink_' + str(arm_idx) + '_' + str(l),
                                                                      curr_idx,[0,0,0],midPt,[self.robot_link_radius,dis])
                    cylinder.type = 'robot_link'
                    self.collision_objects.append(cylinder)

        self.set_rviz_ids()

    def update_all_transforms(self, all_frames):

        # positions = frames[0]
        # rotations = frames[1]
        positions = []
        rotations = []
        for f in all_frames:
            for i,p in enumerate(f[0]):
                positions.append(f[0][i])
                rotations.append(f[1][i])

        for c in self.collision_objects:
            if c.type == 'robot_link':
                name = c.name
                name_arr = name.split('_')
                arm_id = int(name_arr[1])
                link_id = int(name_arr[2])
                ptA = all_frames[arm_id][0][link_id]
                ptB = all_frames[arm_id][0][link_id+1]
                midPt = ptA + 0.5 * (ptB - ptA)
                final_pos = midPt

                rot_mat = np.zeros((3,3))
                z = ptB - ptA
                norm = max(np.linalg.norm(z), 0.000001)
                z = (1.0/ norm)* z
                up = np.array([0,0,1])
                if np.dot(z, up) == 1.0:
                    up = np.array([1,0,0])
                x = np.cross(up, z)
                y = np.cross(z,x)
                rot_mat[:,0] = x
                rot_mat[:,1] = y
                rot_mat[:,2] = z

                final_quat = quaternion_from_matrix(rot_mat)
            else:
                coordinate_frame = c.coordinate_frame
                # first, do local transforms
                frame_len = len(positions)
                if coordinate_frame == 0:
                    rot_mat = rotations[0]
                    final_pos = positions[0]
                elif coordinate_frame >= frame_len:
                    rot_mat = rotations[frame_len-1]
                    final_pos = positions[frame_len-1]
                else:
                    rot_mat = rotations[coordinate_frame]
                    final_pos = positions[coordinate_frame-1]

                final_quat = quaternion_from_matrix(rot_mat)

                local_translation = np.array(c.translation)
                rotated_local_translation = np.dot(rot_mat, local_translation)
                final_pos = final_pos + rotated_local_translation

                local_rotation = c.quaternion
                final_quat = quaternion_multiply(local_rotation, final_quat)

            c.update_transform(final_pos, final_quat)

    def draw_all(self):
        for c in self.collision_objects:
            if c.__class__ == Collision_Mesh: continue
            else: c.draw_rviz(self.publisher)

    def __str__(self): return str([str(c.name) for c in self.collision_objects])


class Collision_Object:
    def __init__(self, collision_dict):
        self.name = collision_dict['name']
        self.coordinate_frame = collision_dict['coordinate_frame']
        self.rotation = collision_dict['rotation']
        rx, ry, rz = self.rotation[0], self.rotation[1], self.rotation[2]
        self.quaternion = quaternion_from_euler(rx, ry, rz)
        self.translation = collision_dict['translation']
        self.params = collision_dict['parameters']
        self.id = 0
        self.type = ''
        self.make_rviz_marker_super()

    @classmethod
    def init_with_arguments(self, name, coordinate_frame, rotation, translation, params):
        collision_dict = {'name': name, 'coordinate_frame': coordinate_frame, 'rotation': rotation, 'translation': translation, 'parameters': params}
        return self(collision_dict)

    def update_transform(self, translation, rotation):
        if len(translation) == 1:
            translation = translation[0]
        self.t = fcl.Transform(rotation, translation)
        self.obj.setTransform(self.t)
        self.marker.pose.position.x = translation[0]
        self.marker.pose.position.y = translation[1]
        self.marker.pose.position.z = translation[2]
        self.marker.pose.orientation.w = rotation[0]
        self.marker.pose.orientation.x = rotation[1]
        self.marker.pose.orientation.y = rotation[2]
        self.marker.pose.orientation.z = rotation[3]

    def update_rviz_color(self, r, g, b, a):
        self.marker.color.r = float(r)
        self.marker.color.g = float(g)
        self.marker.color.b = float(b)
        self.marker.color.a = float(a)

    def make_rviz_marker_super(self):
        self.marker = Marker()
        self.marker.header.frame_id = 'common_world'
        self.marker.id = self.id
        self.marker.color.a = 0.4
        self.marker.color.g = 1.0
        self.marker.color.b = 0.7
        self.marker.text = self.name

    def make_rviz_marker(self): pass

    def draw_rviz(self,publisher):
        publisher.publish(self.marker)


class Collision_Box(Collision_Object):
    def __init__(self, collision_dict):
        Collision_Object.__init__(self, collision_dict)
        if not len(self.params) == 3:
            raise TypeError(bc.FAIL + 'ERROR: parameters for collision box must be list of 3 floats.' + bc.ENDC)
        self.x = self.params[0]
        self.y = self.params[1]
        self.z = self.params[2]
        self.g = fcl.Box(self.x, self.y, self.z)
        self.t = fcl.Transform()
        self.obj = fcl.CollisionObject(self.g, self.t)
        self.make_rviz_marker()

    def make_rviz_marker(self):
        self.marker.type = self.marker.CUBE
        self.marker.scale.x = float(self.x)
        self.marker.scale.y = float(self.y)
        self.marker.scale.z = float(self.z)

class Collision_Sphere(Collision_Object):
    def __init__(self, collision_dict):
        Collision_Object.__init__(self, collision_dict)
        if not type(self.params) == float:
            raise TypeError(bc.FAIL + 'ERROR: parameters for collision sphere must be a float value (for radius).' + bc.ENDC)

        self.r = self.params
        self.g = fcl.Sphere(self.r)
        self.t = fcl.Transform()
        self.obj = fcl.CollisionObject(self.g, self.t)
        self.make_rviz_marker()

    def make_rviz_marker(self):
        self.marker.type = self.marker.SPHERE
        self.marker.scale.x = float(self.r*2)
        self.marker.scale.y = float(self.r*2)
        self.marker.scale.z = float(self.r*2)

class Collision_Ellipsoid(Collision_Object):
    def __init__(self, collision_dict):
        Collision_Object.__init__(self, collision_dict)
        if not len(self.params) == 3:
            raise TypeError(bc.FAIL + 'ERROR: parameters for collision ellipsoid must be list of 3 floats.' + bc.ENDC)

        self.x, self.y, self.z = self.params[0], self.params[1], self.params[2]
        self.g = fcl.Ellipsoid(self.x, self.y, self.z)
        self.t = fcl.Transform()
        self.obj = fcl.CollisionObject(self.g, self.t)
        self.make_rviz_marker()

    def make_rviz_marker(self):
        self.marker.type = self.marker.SPHERE
        self.marker.scale.x = float(self.x*2)
        self.marker.scale.y = float(self.y*2)
        self.marker.scale.z = float(self.z*2)


class Collision_Capsule(Collision_Object):
    def __init__(self, collision_dict):
        Collision_Object.__init__(self, collision_dict)
        if not len(self.params) == 2:
            raise TypeError(bc.FAIL + 'ERROR: parameters for collision capsule must be list of 2 floats.' + bc.ENDC)

        self.r, self.lz = self.params[0], self.params[1]
        self.g = fcl.Capsule(self.r, self.lz)
        self.t = fcl.Transform()
        self.obj = fcl.CollisionObject(self.g, self.t)
        self.make_rviz_marker()

    def make_rviz_marker(self):
        self.marker.type = self.marker.CYLINDER
        self.marker.scale.x = float(self.r*2)
        self.marker.scale.y = float(self.r*2)
        self.marker.scale.z = float(self.lz)


class Collision_Cone(Collision_Object):
    def __init__(self, collision_dict):
        Collision_Object.__init__(self, collision_dict)
        if not len(self.params) == 2:
            raise TypeError(bc.FAIL + 'ERROR: parameters for collision cone must be list of 2 floats.' + bc.ENDC)

        self.r, self.lz = self.params[0], self.params[1]
        self.g = fcl.Capsule(self.r, self.lz)
        self.t = fcl.Transform()
        self.obj = fcl.CollisionObject(self.g, self.t)
        self.make_rviz_marker()

    def make_rviz_marker(self):
        self.marker.type = self.marker.CYLINDER
        self.marker.scale.x = float(self.r*2)
        self.marker.scale.y = float(self.r*2)
        self.marker.scale.z = float(self.lz)


class Collision_Cylinder(Collision_Object):
    def __init__(self, collision_dict):
        Collision_Object.__init__(self, collision_dict)
        if not len(self.params) == 2:
            raise TypeError(bc.FAIL + 'ERROR: parameters for collision cylinder must be list of 2 floats.' + bc.ENDC)

        self.r, self.lz = self.params[0], self.params[1]
        self.g = fcl.Cylinder(self.r, self.lz)
        self.t = fcl.Transform()
        self.obj = fcl.CollisionObject(self.g, self.t)
        self.make_rviz_marker()

    def make_rviz_marker(self):
        self.marker.type = self.marker.CYLINDER
        self.marker.scale.x = float(self.r*2)
        self.marker.scale.y = float(self.r*2)
        self.marker.scale.z = float(self.lz)


class Collision_Mesh(Collision_Object):
    def __init__(self, collision_dict):
        Collision_Object.__init__(self, collision_dict)
        if not len(self.params) == 2:
            raise TypeError(bc.FAIL + 'ERROR: parameters for collision mesh must be a dictionary consisting of a list of verts and tris.' + bc.ENDC)

        if not len(self.params['verts']) == len(self.params['tris']):
            raise TypeError(bc.FAIL + 'ERROR: number of tris must equal the number of verts in collision mesh.' + bc.ENDC)

        self.verts = np.array(self.params['verts'])
        self.tris = np.array(self.params['tris'])

        self.g = fcl.BVHModel()
        self.g.beginModel(len(self.verts), len(self.tris))
        self.g.addSubModel(self.verts, self.tris)
        self.g.endModel()
        self.t = fcl.Transform()
        self.obj = fcl.CollisionObject(self.g, self.t)
        self.make_rviz_marker()

    def make_rviz_marker(self):
        print(bc.WARNING + 'WARNING: Mesh collision object not supported in rviz visualization' + bc.ENDC)
        self.marker.scale.x = 0.0001
        self.marker.scale.y = 0.0001
        self.marker.scale.z = 0.0001
