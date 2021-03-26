import {
  JOINT_OBJECTIVES,
  CARTESIAN_OBJECTIVES,
  PAIRED_OBJECTIVES
} from './Categories';
import {
  COLORS_RGBA
} from './Colors';
import {quaternionFromEuler} from './Geometry';

const EULER_COMBOS = [[1,0,0],[0,0,1],//[0,0,1],[1,1,0],[0,1,1],[1,0,1],[1,1,1],
                      [-1,0,0],[0,0,-1],//[0,0,-1],[-1,-1,0],[0,-1,-1],
                      [1,0,-1],[-1,0,1],
                      [1,0,1],[-1,0,-1]//[-1,0,-1],[-1,-1,-1],[1,-1,0],[-1,1,0],[0,1,-1],[0,-1,1],[1,0,-1],[-1,0,1],[1,-1,1],[-1,1,1],[1,1,-1],[1,-1,-1],
                      //[-1,-1,1],[-1,1,-1]
                      ];

export function getObjectiveMarkers(props) {
  const { objective, goal, tree, poses, jointNames, jointOrdering, eeFixedJoints, fixedFrame } = props;
  let markers = {}

  const isJointObjective = JOINT_OBJECTIVES.indexOf(objective.variant) >= 0
  const isCartesianObjective = CARTESIAN_OBJECTIVES.indexOf(objective.variant) >= 0
  const isPairedObjective = PAIRED_OBJECTIVES.indexOf(objective.variant) >= 0

  if (isCartesianObjective) {
    let jointName1 = null;
    if (objective.indices[1] === jointNames[objective.indices[0]].length) {
      jointName1 = eeFixedJoints[objective.indices[0]];
    } else {
      jointName1 = jointNames[objective.indices[0]][objective.indices[1]]
    }
    let frame1 = tree.joints[jointName1].child;
    markers.gui_marker_objective_localizer_1 = {
      frame_id:`/${frame1}`,
      pose:{position:{x:0,y:0,z:0},
            orientation:{w:1,x:0,y:0,z:0}
           },
      scale:{x:0.08,y:0.08,z:0.08},
      type:'sphere',
      color:COLORS_RGBA.PRIMARY_POINT
    }
    if (isPairedObjective) {
      let jointName2 = null;
      if (objective.indices[1] === jointNames[objective.indices[0]].length) {
        jointName2 = eeFixedJoints[objective.indices[0]];
      } else {
        jointName2 = jointNames[objective.indices[0]][objective.indices[1]]
      }
      let frame2 = tree.joints[jointName2].child;
      markers.gui_marker_objective_localizer_2 = {
        frame_id:`/${frame2}`,
        pose:{position:{x:0,y:0,z:0},
              orientation:{w:1,x:0,y:0,z:0}
             },
        scale:{x:0.08,y:0.08,z:0.08},
        type:'sphere',
        color:COLORS_RGBA.SECONDARY_POINT
      }
    }
  } else if (isJointObjective) {
    let frame1 = tree.joints[jointOrdering[objective.indices[0]]].child;
    markers.gui_marker_objective_localizer_1 = {
      frame_id:`/${frame1}`,
      pose:{position:{x:0,y:0,z:0},
            orientation:{w:1,x:0,y:0,z:0}
           },
      scale:{x:0.08,y:0.08,z:0.08},
      type:'sphere',
      color:COLORS_RGBA.PRIMARY_JOINT
    }
    if (isPairedObjective) {
      let frame2 = tree.joints[jointOrdering[objective.indices[1]]].child;
      markers.gui_marker_objective_localizer_2 = {
        frame_id:`/${frame2}`,
        pose:{position:{x:0,y:0,z:0},
              orientation:{w:1,x:0,y:0,z:0}
             },
        scale:{x:0.08,y:0.08,z:0.08},
        type:'sphere',
        color:COLORS_RGBA.SECONDARY_JOINT
      }
    }
  }

  if (objective.variant === 'position_liveliness') {
    // Use the position from the current pose
    let position = poses[objective.indices[0]][objective.indices[1]].position
    let color = {...COLORS_RGBA.PRIMARY_POINT};
    color.a = .4;
    markers.active_objective_visualizer = {
      frame_id:`/${fixedFrame}`,
      pose:{position:position,
            orientation:{w:1,x:0,y:0,z:0}
           },
      scale:{x:objective.shape[0],y:objective.shape[1],z:objective.shape[2]},
      type:'sphere',
      color:color
    }

  } else if (objective.variant === 'orientation_liveliness') {
    // Use the position, orientation from the current pose
    let position = poses[objective.indices[0]][objective.indices[1]].position;
    let base_rotation = poses[objective.indices[0]][objective.indices[1]].rotation;
    let color = {...COLORS_RGBA.PRIMARY_POINT};
    color.a = .4;
    EULER_COMBOS.forEach((combo,idx)=>{
      let rotation = {...base_rotation};
      rotation = [
        rotation.r+combo[0]*objective.shape[0],
        rotation.p+combo[1]*objective.shape[1],
        rotation.y+combo[2]*objective.shape[2],
      ];
      let orientation = quaternionFromEuler(rotation);
      markers[`active_objective_visualizer_${idx}`] = {
        frame_id:`/${fixedFrame}`,
        pose:{position:position,
              orientation:{w:orientation[0],x:orientation[1],y:orientation[2],z:orientation[3]}
             },
        scale:{x:0.2,y:0.01,z:0.01},
        type:'arrow',
        color:color
      }
    })



  } else if (objective.variant === 'relative_motion_liveliness') {


  } else if (objective.variant === 'base_link_position_liveliness') {

  }

  if (goal.vector !== undefined) {

  } else if (goal.quaternion !== undefined) {

  } else if (goal.scalar !== undefined) {

  } else if (goal.pose !== undefined) {

  }

  return markers
}
