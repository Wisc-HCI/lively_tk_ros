const BASE_OBJECTIVES = ['joint_limits','nn_collision','env_collision','min_velocity','min_acceleration','min_jerk'];
const DIRECTION_OBJECTIVES = ['position_match','orientation_match','position_mirroring','orientation_mirroring',
                              'position_bounding','orientation_bounding','joint_mirroring','joint_match'];
const LIVELINESS_OBJECTIVES = ['position_liveliness','orientation_liveliness','joint_liveliness',
                               'base_link_position_liveliness','relative_motion_liveliness'];
const CARTESIAN_OBJECTIVES = ['position_match','orientation_match','position_mirroring','orientation_mirroring',
                              'position_bounding','orientation_bounding','position_liveliness','orientation_liveliness',
                              'relative_motion_liveliness'];
const JOINT_OBJECTIVES = ['joint_mirroring','joint_match','joint_liveliness'];
const PAIRED_OBJECTIVES = ['position_mirroring','orientation_mirroring','joint_mirroring','relative_motion_liveliness'];
const BEHAVIOR_ATTRIBUTE_GROUPS = [BASE_OBJECTIVES,DIRECTION_OBJECTIVES,LIVELINESS_OBJECTIVES];
const BEHAVIOR_ATTRIBUTE_GROUP_NAMES = ['Base','Directions','Liveliness'];


export {BASE_OBJECTIVES,
        DIRECTION_OBJECTIVES,
        LIVELINESS_OBJECTIVES,
        CARTESIAN_OBJECTIVES,
        JOINT_OBJECTIVES,
        PAIRED_OBJECTIVES,
        BEHAVIOR_ATTRIBUTE_GROUPS,
        BEHAVIOR_ATTRIBUTE_GROUP_NAMES};
