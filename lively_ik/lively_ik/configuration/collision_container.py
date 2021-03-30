import fcl
import numpy as np
import lively_ik.configuration.transformations as T

class CollisionObjectContainer:
    def __init__(self,config):
        self.collision_objects = []
        self.robot_link_radius = config['robot_link_radius']
        self.joint_names = config['joint_names']
        self.tree = config['robot_tree']

        for sphere in config['static_environment']['spheres']:
            self.collision_objects.append(CollisionSphere(sphere))
        for cuboid in config['static_environment']['cuboids']:
            self.collision_objects.append(CollisionBox(cuboid))

    def get_min_distance(self, a, b):
        obja = self.collision_objects[a].obj
        objb = self.collision_objects[b].obj

        self.request = fcl.DistanceRequest()
        self.result = fcl.DistanceResult()

        ret = fcl.distance(obja, objb, self.request, self.result)
        return self.result.min_distance

    def add_collision_objects_from_robot(self, robot, exclusion=[]):
        numDOF = robot.numDOF

        frames_list = robot.getFrames([0,0,0],numDOF*[0])
        for arm_idx in range(len(frames_list)):
            frames = frames_list[arm_idx]
            jtPts = frames[0]

            numLinks = (len(jtPts)-1)
            for l in range(numLinks):
                curr_idx = numLinks*arm_idx + l
                if not curr_idx in exclusion:
                    ptA = np.array(jtPts[l])
                    ptB = np.array(jtPts[l+1])
                    midPt = ptA + 0.5*(ptB - ptA)
                    dis = np.linalg.norm(ptA - ptB)
                    if dis < 0.02:
                        continue
                    cylinder = CollisionCylinder({'name':'robotLink_' + str(arm_idx) + '_' + str(l),
                                                  'coordinate_frame':curr_idx,
                                                  'rx':0,'ry':0,'rz':0,
                                                  'tx':midPt[0],'ty':midPt[1],'tz':midPt[2],
                                                  'radius':self.robot_link_radius,
                                                  'lz':dis})
                    cylinder.type = 'robot_link'
                    self.collision_objects.append(cylinder)

    def update_all_transforms(self, transform, all_frames):

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
                ptA = np.array(all_frames[arm_id][0][link_id])
                ptB = np.array(all_frames[arm_id][0][link_id+1])
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
                arm_idx = 0
                joint_idx = 0

                final_pos = np.array(transform)
                final_quat = np.array([1,0,0,0])
                rot_mat = np.identity(3)
                for arm_idx,arm in enumerate(self.joint_names):
                    for joint_idx,joint in enumerate(arm):
                        if self.tree['joints'][joint]['child'] == coordinate_frame:
                            final_pos = all_frames[arm_idx][0][joint_idx]
                            rot_mat = all_frames[arm_idx][1][joint_idx]
                            final_quat = T.quaternion_from_matrix(rot_mat)
                            break

                local_translation = np.array(c.translation)
                rotated_local_translation = np.dot(rot_mat, local_translation)
                final_pos = final_pos + rotated_local_translation

                local_rotation = c.quaternion
                final_quat = T.quaternion_multiply(local_rotation, final_quat)

            c.update_transform(final_pos, final_quat)


    def __str__(self): return str([str(c.name) for c in self.collision_objects])


class CollisionObject:
    def __init__(self, name, coordinate_frame, tx, ty, tz):
        self.name = name
        self.coordinate_frame = coordinate_frame
        self.translation = [tx, ty, tz]
        self.quaternion = T.quaternion_from_euler(0, 0, 0)
        self.type = None

    def update_transform(self, translation, rotation):
        if len(translation) == 1:
            translation = translation[0]
        self.t = fcl.Transform(rotation, translation)
        self.obj.setTransform(self.t)


class CollisionBox(CollisionObject):
    def __init__(self, collision_dict):
        CollisionObject.__init__(self, collision_dict['name'],collision_dict['coordinate_frame'],
                                 collision_dict['tx'],collision_dict['ty'],collision_dict['tz'])
        self.quaternion = T.quaternion_from_euler(collision_dict['rx'], collision_dict['ry'], collision_dict['rz'])
        self.x = collision_dict['x_halflength']*2
        self.y = collision_dict['y_halflength']*2
        self.z = collision_dict['z_halflength']*2
        self.g = fcl.Box(self.x, self.y, self.z)
        self.t = fcl.Transform()
        self.obj = fcl.CollisionObject(self.g, self.t)

class CollisionSphere(CollisionObject):
    def __init__(self, collision_dict):
        CollisionObject.__init__(self, collision_dict['name'],collision_dict['coordinate_frame'],
                                 collision_dict['tx'],collision_dict['ty'],collision_dict['tz'])
        self.g = fcl.Sphere(collision_dict['radius'])
        self.t = fcl.Transform()
        self.obj = fcl.CollisionObject(self.g, self.t)

class CollisionCylinder(CollisionObject):
    def __init__(self, collision_dict):
        CollisionObject.__init__(self, collision_dict['name'],collision_dict['coordinate_frame'],
                                 collision_dict['tx'],collision_dict['ty'],collision_dict['tz'])
        self.quaternion = T.quaternion_from_euler(collision_dict['rx'], collision_dict['ry'], collision_dict['rz'])
        self.r = collision_dict['radius']
        self.lz = collision_dict['lz']
        self.g = fcl.Cylinder(self.r, self.lz)
        self.t = fcl.Transform()
        self.obj = fcl.CollisionObject(self.g, self.t)
