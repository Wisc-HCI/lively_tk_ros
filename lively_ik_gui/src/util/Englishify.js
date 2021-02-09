import React from 'react';

const getObjectivePreview = (objectiveData, fixedFrame, eeFixedJoints, jointOrdering) => {
  switch (objectiveData.variant) {
    case 'joint_limits':
      return 'Ensure joints remain within their limits'
    case 'nn_collision':
      return 'Ensure robot doesn\'t collide with the static environment'
    case 'env_collision':
      return 'Ensure robot doesn\'t collide with the dynamic environment'
    case 'min_velocity':
      return 'Ensure joints don\'t move too quickly'
    case 'min_acceleration':
      return 'Ensure joints don\'t accelerate too quickly'
    case 'min_jerk':
      return 'Ensure joints don\'t jerk too quickly'
    case 'ee_position_match':
      return <span>
              <span>Make the </span>
              <span style={{backgroundColor:'#1890ff',color:'white',padding:4,borderRadius:5}}>
                {eeFixedJoints[objectiveData.index]}
              </span>
              <span> have a given position</span>
             </span>
    case 'ee_orientation_match':
      return <span>
              <span>Make the </span>
              <span style={{backgroundColor:'#1890ff',color:'white',padding:4,borderRadius:5}}>
                {eeFixedJoints[objectiveData.index]}
              </span>
              <span> have a given rotation</span>
             </span>
    case 'joint_match':
      return <span>
              <span>Make the </span>
              <span style={{backgroundColor:'#BE33FF',color:'white',padding:4,borderRadius:5}}>
               {jointOrdering[objectiveData.index]}
              </span>
              <span> have a given value</span>
            </span>
    case 'ee_position_mirroring':
      return <span>
              <span>Match the positions of </span>
              <span style={{backgroundColor:'#1890ff',color:'white',padding:4,borderRadius:5}}>
                {eeFixedJoints[objectiveData.index]}
              </span>
              <span> and </span>
              <span style={{backgroundColor:'#1890ff',color:'white',padding:4,borderRadius:5}}>
                {eeFixedJoints[objectiveData.secondary_index]}
              </span>
             </span>
    case 'ee_orientation_mirroring':
      return <span>
              <span>Match the rotations of </span>
              <span style={{backgroundColor:'#1890ff',color:'white',padding:4,borderRadius:5}}>
                {eeFixedJoints[objectiveData.index]}
              </span>
              <span> and </span>
              <span style={{backgroundColor:'#1890ff',color:'white',padding:4,borderRadius:5}}>
                {eeFixedJoints[objectiveData.secondary_index]}
              </span>
             </span>
    case 'ee_position_bounding':
      return <span>
              <span>Make the position of</span>
              <span style={{backgroundColor:'#1890ff',color:'white',padding:4,borderRadius:5}}>
                {eeFixedJoints[objectiveData.index]}
              </span>
              <span> stay within a certain range</span>
             </span>
    case 'ee_orientation_bounding':
      return <span>
              <span>Make the orientation of</span>
              <span style={{backgroundColor:'#1890ff',color:'white',padding:4,borderRadius:5}}>
                {eeFixedJoints[objectiveData.index]}
              </span>
              <span> stay within a certain range</span>
             </span>
    case 'joint_mirroring':
      return <span>
              <span>Match the values of </span>
              <span style={{backgroundColor:'#BE33FF',color:'white',padding:4,borderRadius:5}}>
                {jointOrdering[objectiveData.index]}
              </span>
              <span> and </span>
              <span style={{backgroundColor:'#BE33FF',color:'white',padding:4,borderRadius:5}}>
                {jointOrdering[objectiveData.secondary_index]}
              </span>
             </span>
    case 'ee_position_liveliness':
      return <span>
              <span>Make the position of </span>
              <span style={{backgroundColor:'#1890ff',color:'white',padding:4,borderRadius:5}}>
                {eeFixedJoints[objectiveData.index]}
              </span>
              <span> have lifelike motion</span>
             </span>
    case 'ee_orientation_liveliness':
      return <span>
              <span>Make the rotation of </span>
              <span style={{backgroundColor:'#1890ff',color:'white',padding:4,borderRadius:5}}>
                {eeFixedJoints[objectiveData.index]}
              </span>
              <span> have lifelike motion</span>
             </span>
    case 'joint_liveliness':
      return <span>
              <span>Make the value of </span>
              <span style={{backgroundColor:'#BE33FF',color:'white',padding:4,borderRadius:5}}>
                {jointOrdering[objectiveData.index]}
              </span>
              <span> have lifelike motion</span>
             </span>
    case 'base_link_position_liveliness':
      return <span>
            <span>Make the robot root </span>
            <span style={{backgroundColor:'#E73091',color:'white',padding:4,borderRadius:5}}>
              {fixedFrame}
            </span>
            <span> have lifelike motion</span>
           </span>
    default:
      return <></>
  }
}

export {getObjectivePreview}
