// const BASE_OBJECTIVES = ['macro_smoothness','joint_limits','nn_collision','env_collision','min_velocity','min_acceleration','min_jerk'];
// const DIRECTION_OBJECTIVES = ['position_match','orientation_match','position_mirroring','orientation_mirroring',
//                               'position_bounding','joint_mirroring','joint_match'];
// const LIVELINESS_OBJECTIVES = ['position_liveliness','orientation_liveliness','joint_liveliness',
//                                'base_link_position_liveliness','relative_motion_liveliness','gravity'];
// const CARTESIAN_OBJECTIVES = ['position_match','orientation_match','position_mirroring','orientation_mirroring',
//                               'position_bounding','position_liveliness','orientation_liveliness',
//                               'relative_motion_liveliness','gravity'];
// const JOINT_OBJECTIVES = ['joint_mirroring','joint_match','joint_liveliness','joint_mirroring'];
// const PAIRED_OBJECTIVES = ['position_mirroring','orientation_mirroring','joint_mirroring','relative_motion_liveliness'];
// const BEHAVIOR_ATTRIBUTE_GROUPS = [BASE_OBJECTIVES,DIRECTION_OBJECTIVES,LIVELINESS_OBJECTIVES];
// const BEHAVIOR_ATTRIBUTE_GROUP_NAMES = ['Base','Directions','Liveliness'];
const BASE_OBJECTIVES = ['macro_smoothness','joint_limits','min_velocity','min_acceleration','min_jerk'];
const DIRECTION_OBJECTIVES = ['position_match','orientation_match','position_mirroring','orientation_mirroring',
                              'joint_mirroring','joint_match','base_link_position_match','distance_match'];
const LIVELINESS_OBJECTIVES = ['position_liveliness','orientation_liveliness','joint_liveliness',
                               'base_link_position_liveliness','relative_motion_liveliness','gravity'];
const CARTESIAN_OBJECTIVES = ['position_match','orientation_match','position_mirroring','orientation_mirroring',
                              'position_liveliness','orientation_liveliness',
                              'relative_motion_liveliness','gravity','distance_match'];
const JOINT_OBJECTIVES = ['joint_mirroring','joint_match','joint_liveliness','joint_mirroring'];
const PAIRED_OBJECTIVES = ['position_mirroring','orientation_mirroring','joint_mirroring','relative_motion_liveliness','distance_match'];
const BEHAVIOR_ATTRIBUTE_GROUPS = [BASE_OBJECTIVES,DIRECTION_OBJECTIVES,LIVELINESS_OBJECTIVES];
const BEHAVIOR_ATTRIBUTE_GROUP_NAMES = ['Base','Control','Liveliness'];


export {BASE_OBJECTIVES,
        DIRECTION_OBJECTIVES,
        LIVELINESS_OBJECTIVES,
        CARTESIAN_OBJECTIVES,
        JOINT_OBJECTIVES,
        PAIRED_OBJECTIVES,
        BEHAVIOR_ATTRIBUTE_GROUPS,
        BEHAVIOR_ATTRIBUTE_GROUP_NAMES};
