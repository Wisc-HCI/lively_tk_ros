import fcl
import numpy as np
import lively_ik.configuration.transformations as T

class CollisionObjectContainer:
    def __init__(self, environment_spec, robot_link_radius=0.05):
        self.collision_objects = []
        self.robot_link_radius = robot_link_radius

        for sphere in environment_spec['spheres']:
            self.collision_objects.append(Collision_Sphere(sphere))
        for cuboid in environment_spec['cuboids']:
            self.collision_objects.append(Collision_Box(cuboid))

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
                    cylinder = Collision_Cylinder({'name':'robotLink_' + str(arm_idx) + '_' + str(l),
                                                   'coordinate_frame':curr_idx,
                                                   'rx':0,'ry':0,'rz':0,
                                                   'tx':midPt[0],'ty':midPt[1],'tz':midPt[2],
                                                   'radius':self.robot_link_radius,
                                                   'lz':dis})
                    cylinder.type = 'robot_link'
                    self.collision_objects.append(cylinder)

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
                final_quat = T.quaternion_from_matrix(rot_mat)
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

                final_quat = T.quaternion_from_matrix(rot_mat)

                local_translation = np.array(c.translation)
                rotated_local_translation = np.dot(rot_mat, local_translation)
                final_pos = final_pos + rotated_local_translation

                local_rotation = c.quaternion
                final_quat = T.quaternion_multiply(local_rotation, final_quat)

            c.update_transform(final_pos, final_quat)


    def __str__(self): return str([str(c.name) for c in self.collision_objects])


class Collision_Object:
    def __init__(self, collision_dict):
        self.name = collision_dict['name']
        self.coordinate_frame = collision_dict['coordinate_frame']
        self.rotation = [collision_dict['rx'], collision_dict['ry'], collision_dict['rz']]
        self.quaternion = T.quaternion_from_euler(collision_dict['rx'], collision_dict['ry'], collision_dict['rz'])
        self.translation = [collision_dict['tx'], collision_dict['ty'], collision_dict['tz']]
        self.id = 0
        self.type = ''

    def update_transform(self, translation, rotation):
        if len(translation) == 1:
            translation = translation[0]
        self.t = fcl.Transform(rotation, translation)
        self.obj.setTransform(self.t)


class Collision_Box(Collision_Object):
    def __init__(self, collision_dict):
        Collision_Object.__init__(self, collision_dict)

        self.x = collision_dict['x_halflength']*2
        self.y = collision_dict['y_halflength']*2
        self.z = collision_dict['z_halflength']*2
        self.g = fcl.Box(self.x, self.y, self.z)
        self.t = fcl.Transform()
        self.obj = fcl.CollisionObject(self.g, self.t)

class Collision_Sphere(Collision_Object):
    def __init__(self, collision_dict):
        Collision_Object.__init__(self, collision_dict)
        self.g = fcl.Sphere(collision_dict['radius'])
        self.t = fcl.Transform()
        self.obj = fcl.CollisionObject(self.g, self.t)

class Collision_Cylinder(Collision_Object):
    def __init__(self, collision_dict):
        Collision_Object.__init__(self, collision_dict)
        self.r = collision_dict['radius']
        self.lz = collision_dict['lz']
        self.g = fcl.Cylinder(self.r, self.lz)
        self.t = fcl.Transform()
        self.obj = fcl.CollisionObject(self.g, self.t)
