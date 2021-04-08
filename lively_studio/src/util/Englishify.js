import React from 'react';
import {
  COLORS_HEX
} from './Colors';

const getObjectivePreview = (objectiveData, fixedFrame, eeFixedJoints, jointOrdering, jointNames) => {
  let jointName1 = '';
  let jointName2 = '';
  switch (objectiveData.variant) {
    case 'macro_smoothness':
      return 'General Smoothness that incorporates velocity, acceleration, and jerk'
    case 'joint_limits':
      return 'Ensure joints remain within their limits'
    case 'nn_collision':
      return <span>{"Ensure robot doesn't collide with the "}
              <span style={{backgroundColor:COLORS_HEX.COLLISION_OBJECT, color:'white', padding:4,borderRadius:5}}>
                {'static environment'}
              </span>
            </span>
    case 'env_collision':
      return 'Ensure robot doesn\'t collide with the dynamic environment'
    case 'min_velocity':
      return 'Ensure joints don\'t move too quickly'
    case 'min_acceleration':
      return 'Ensure joints don\'t accelerate too quickly'
    case 'min_jerk':
      return 'Ensure joints don\'t jerk too quickly'
    case 'position_match':
      if (objectiveData.indices[1] >= jointNames[objectiveData.indices[0]].length) {
        jointName1 = eeFixedJoints[objectiveData.indices[0]]
      } else {
        jointName1 = jointNames[objectiveData.indices[0]][objectiveData.indices[1]]
      }
      return <span>
              <span>Make the </span>
              <span style={{backgroundColor:COLORS_HEX.PRIMARY_POINT,color:'white',padding:4,borderRadius:5}}>
                {jointName1}
              </span>
              <span> have a given position</span>
             </span>
    case 'orientation_match':
      if (objectiveData.indices[1] >= jointNames[objectiveData.indices[0]].length) {
        jointName1 = eeFixedJoints[objectiveData.indices[0]]
      } else {
        jointName1 = jointNames[objectiveData.indices[0]][objectiveData.indices[1]]
      }
      return <span>
              <span>Make the </span>
              <span style={{backgroundColor:COLORS_HEX.PRIMARY_POINT,color:'white',padding:4,borderRadius:5}}>
                {jointName1}
              </span>
              <span> have a given rotation</span>
             </span>
    case 'joint_match':
      return <span>
              <span>Make the </span>
              <span style={{backgroundColor:COLORS_HEX.PRIMARY_JOINT,color:'white',padding:4,borderRadius:5}}>
               {jointOrdering[objectiveData.indices[0]]}
              </span>
              <span> have a given value</span>
            </span>
    case 'position_mirroring':
      if (objectiveData.indices[1] >= jointNames[objectiveData.indices[0]].length) {
        jointName1 = eeFixedJoints[objectiveData.indices[0]]
      } else {
        jointName1 = jointNames[objectiveData.indices[0]][objectiveData.indices[1]]
      }
      if (objectiveData.indices[3] >= jointNames[objectiveData.indices[2]].length) {
        jointName2 = eeFixedJoints[objectiveData.indices[2]]
      } else {
        jointName2 = jointNames[objectiveData.indices[2]][objectiveData.indices[3]]
      }
      return <span>
              <span>Match the positions of </span>
              <span style={{backgroundColor:COLORS_HEX.PRIMARY_POINT,color:'white',padding:4,borderRadius:5}}>
                {jointName1}
              </span>
              <span> and </span>
              <span style={{backgroundColor:COLORS_HEX.SECONDARY_POINT,color:'white',padding:4,borderRadius:5}}>
                {jointName2}
              </span>
             </span>
    case 'orientation_mirroring':
      if (objectiveData.indices[1] >= jointNames[objectiveData.indices[0]].length) {
        jointName1 = eeFixedJoints[objectiveData.indices[0]]
      } else {
        jointName1 = jointNames[objectiveData.indices[0]][objectiveData.indices[1]]
      }
      if (objectiveData.indices[3] >= jointNames[objectiveData.indices[2]].length) {
        jointName2 = eeFixedJoints[objectiveData.indices[2]]
      } else {
        jointName2 = jointNames[objectiveData.indices[2]][objectiveData.indices[3]]
      }
      return <span>
              <span>Match the rotations of </span>
              <span style={{backgroundColor:COLORS_HEX.PRIMARY_POINT,color:'white',padding:4,borderRadius:5}}>
                {jointName1}
              </span>
              <span> and </span>
              <span style={{backgroundColor:COLORS_HEX.SECONDARY_POINT,color:'white',padding:4,borderRadius:5}}>
                {jointName2}
              </span>
             </span>
    case 'position_bounding':
      if (objectiveData.indices[1] >= jointNames[objectiveData.indices[0]].length) {
        jointName1 = eeFixedJoints[objectiveData.indices[0]]
      } else {
        jointName1 = jointNames[objectiveData.indices[0]][objectiveData.indices[1]]
      }
      return <span>
              <span>Make the position of</span>
              <span style={{backgroundColor:COLORS_HEX.PRIMARY_POINT,color:'white',padding:4,borderRadius:5}}>
                {jointName1}
              </span>
              <span> stay within a certain range</span>
             </span>
    case 'orientation_bounding':
      if (objectiveData.indices[1] >= jointNames[objectiveData.indices[0]].length) {
        jointName1 = eeFixedJoints[objectiveData.indices[0]]
      } else {
        jointName1 = jointNames[objectiveData.indices[0]][objectiveData.indices[1]]
      }
      return <span>
              <span>Make the orientation of</span>
              <span style={{backgroundColor:COLORS_HEX.PRIMARY_POINT,color:'white',padding:4,borderRadius:5}}>
                {jointName1}
              </span>
              <span> stay within a certain range</span>
             </span>
    case 'joint_mirroring':
      return <span>
              <span>Match the values of </span>
              <span style={{backgroundColor:COLORS_HEX.PRIMARY_JOINT,color:'white',padding:4,borderRadius:5}}>
                {jointOrdering[objectiveData.indices[0]]}
              </span>
              <span> and </span>
              <span style={{backgroundColor:COLORS_HEX.SECONDARY_JOINT,color:'white',padding:4,borderRadius:5}}>
                {jointOrdering[objectiveData.indices[1]]}
              </span>
             </span>
    case 'position_liveliness':
      if (objectiveData.indices[1] >= jointNames[objectiveData.indices[0]].length) {
        jointName1 = eeFixedJoints[objectiveData.indices[0]]
      } else {
        jointName1 = jointNames[objectiveData.indices[0]][objectiveData.indices[1]]
      }
      return <span>
              <span>Make the position of </span>
              <span style={{backgroundColor:COLORS_HEX.PRIMARY_POINT,color:'white',padding:4,borderRadius:5}}>
                {jointName1}
              </span>
              <span> have lifelike motion</span>
             </span>
    case 'orientation_liveliness':
      if (objectiveData.indices[1] >= jointNames[objectiveData.indices[0]].length) {
        jointName1 = eeFixedJoints[objectiveData.indices[0]]
      } else {
        jointName1 = jointNames[objectiveData.indices[0]][objectiveData.indices[1]]
      }
      return <span>
              <span>Make the rotation of </span>
              <span style={{backgroundColor:COLORS_HEX.PRIMARY_POINT,color:'white',padding:4,borderRadius:5}}>
                {jointName1}
              </span>
              <span> have lifelike motion</span>
             </span>
    case 'joint_liveliness':
      return <span>
              <span>Make the value of </span>
              <span style={{backgroundColor:COLORS_HEX.PRIMARY_JOINT,color:'white',padding:4,borderRadius:5}}>
                {jointOrdering[objectiveData.indices[0]]}
              </span>
              <span> have lifelike motion</span>
             </span>
    case 'relative_motion_liveliness':
      if (objectiveData.indices[1] >= jointNames[objectiveData.indices[0]].length) {
        jointName1 = eeFixedJoints[objectiveData.indices[0]]
      } else {
        jointName1 = jointNames[objectiveData.indices[0]][objectiveData.indices[1]]
      }
      if (objectiveData.indices[3] >= jointNames[objectiveData.indices[2]].length) {
        jointName2 = eeFixedJoints[objectiveData.indices[2]]
      } else {
        jointName2 = jointNames[objectiveData.indices[2]][objectiveData.indices[3]]
      }
      return <span>
              <span>Move position of </span>
              <span style={{backgroundColor:COLORS_HEX.PRIMARY_POINT,color:'white',padding:4,borderRadius:5}}>
                {jointName1}
              </span>
              <span> towards and away from </span>
              <span style={{backgroundColor:COLORS_HEX.SECONDARY_POINT,color:'white',padding:4,borderRadius:5}}>
                {jointName2}
              </span>
             </span>
     case 'distance_match':
       if (objectiveData.indices[1] >= jointNames[objectiveData.indices[0]].length) {
         jointName1 = eeFixedJoints[objectiveData.indices[0]]
       } else {
         jointName1 = jointNames[objectiveData.indices[0]][objectiveData.indices[1]]
       }
       if (objectiveData.indices[3] >= jointNames[objectiveData.indices[2]].length) {
         jointName2 = eeFixedJoints[objectiveData.indices[2]]
       } else {
         jointName2 = jointNames[objectiveData.indices[2]][objectiveData.indices[3]]
       }
       return <span>
               <span>Move position of </span>
               <span style={{backgroundColor:COLORS_HEX.PRIMARY_POINT,color:'white',padding:4,borderRadius:5}}>
                 {jointName1}
               </span>
               <span> to be some distance from </span>
               <span style={{backgroundColor:COLORS_HEX.SECONDARY_POINT,color:'white',padding:4,borderRadius:5}}>
                 {jointName2}
               </span>
              </span>
    case 'base_link_position_liveliness':
      return <span>
            <span>Make the robot root </span>
            <span style={{backgroundColor:COLORS_HEX.PRIMARY_POINT,color:'white',padding:4,borderRadius:5}}>
              {fixedFrame}
            </span>
            <span> have lifelike motion</span>
           </span>
    case 'base_link_position_match':
      return <span>
           <span>Control the position of the robot root </span>
           <span style={{backgroundColor:COLORS_HEX.PRIMARY_POINT,color:'white',padding:4,borderRadius:5}}>
             {fixedFrame}
           </span>
          </span>
    case 'gravity':
      if (objectiveData.indices[1] >= jointNames[objectiveData.indices[0]].length) {
        jointName1 = eeFixedJoints[objectiveData.indices[0]]
      } else {
        jointName1 = jointNames[objectiveData.indices[0]][objectiveData.indices[1]]
      }
      return <span>
              <span>Make the position of </span>
              <span style={{backgroundColor:COLORS_HEX.PRIMARY_POINT,color:'white',padding:4,borderRadius:5}}>
                {jointName1}
              </span>
              <span> be subject to gravity.</span>
             </span>
    default:
      return <></>
  }
}


export {getObjectivePreview};
