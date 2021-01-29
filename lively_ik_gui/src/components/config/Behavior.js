import React from 'react';
import { Input, Select, Collapse, List, Tag } from 'antd';
const { Panel } = Collapse;
const { Option } = Select;

class Behavior extends React.Component {

  getEENameByIdx = (idx) => {
    return this.props.config.ee_fixed_joints[idx]
  }

  getJointNameByIdx = (idx) => {
    return this.props.config.joint_ordering[idx]
  }

  getObjectivePreview = (objectiveData) => {
    switch (objectiveData.objective.variant) {
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
                  {this.getEENameByIdx(objectiveData.objective.index)}
                </span>
                <span> have a given position</span>
               </span>
      case 'ee_orientation_match':
        return <span>
                <span>Make the </span>
                <span style={{backgroundColor:'#1890ff',color:'white',padding:4,borderRadius:5}}>
                  {this.getEENameByIdx(objectiveData.objective.index)}
                </span>
                <span> have a given rotation</span>
               </span>
      case 'joint_match':
        return <span>
                <span>Make the </span>
                <span style={{backgroundColor:'#BE33FF',color:'white',padding:4,borderRadius:5}}>
                 {this.getJointNameByIdx(objectiveData.objective.index)}
                </span>
                <span> have a given value</span>
              </span>
      case 'ee_position_mirroring':
        return <span>
                <span>Match the positions of </span>
                <span style={{backgroundColor:'#1890ff',color:'white',padding:4,borderRadius:5}}>
                  {this.getEENameByIdx(objectiveData.objective.index)}
                </span>
                <span> and </span>
                <span style={{backgroundColor:'#1890ff',color:'white',padding:4,borderRadius:5}}>
                  {this.getEENameByIdx(objectiveData.objective.secondary_index)}
                </span>
               </span>
      case 'ee_orientation_mirroring':
        return <span>
                <span>Match the rotations of </span>
                <span style={{backgroundColor:'#1890ff',color:'white',padding:4,borderRadius:5}}>
                  {this.getEENameByIdx(objectiveData.objective.index)}
                </span>
                <span> and </span>
                <span style={{backgroundColor:'#1890ff',color:'white',padding:4,borderRadius:5}}>
                  {this.getEENameByIdx(objectiveData.objective.secondary_index)}
                </span>
               </span>
      case 'ee_position_bounding':
        return <span>
                <span>Make the position of</span>
                <span style={{backgroundColor:'#1890ff',color:'white',padding:4,borderRadius:5}}>
                  {this.getEENameByIdx(objectiveData.objective.index)}
                </span>
                <span> stay within a certain range</span>
               </span>
      case 'ee_orientation_bounding':
        return <span>
                <span>Make the orientation of</span>
                <span style={{backgroundColor:'#1890ff',color:'white',padding:4,borderRadius:5}}>
                  {this.getEENameByIdx(objectiveData.objective.index)}
                </span>
                <span> stay within a certain range</span>
               </span>
      case 'joint_mirroring':
        return <span>
                <span>Match the values of </span>
                <span style={{backgroundColor:'#BE33FF',color:'white',padding:4,borderRadius:5}}>
                  {this.getJointNameByIdx(objectiveData.objective.index)}
                </span>
                <span> and </span>
                <span style={{backgroundColor:'#BE33FF',color:'white',padding:4,borderRadius:5}}>
                  {this.getJointNameByIdx(objectiveData.objective.secondary_index)}
                </span>
               </span>
      case 'ee_position_liveliness':
        return <span>
                <span>Make the position of </span>
                <span style={{backgroundColor:'#1890ff',color:'white',padding:4,borderRadius:5}}>
                  {this.getEENameByIdx(objectiveData.objective.index)}
                </span>
                <span> have lifelike motion</span>
               </span>
      case 'ee_orientation_liveliness':
        return <span>
                <span>Make the rotation of </span>
                <span style={{backgroundColor:'#1890ff',color:'white',padding:4,borderRadius:5}}>
                  {this.getEENameByIdx(objectiveData.objective.index)}
                </span>
                <span> have lifelike motion</span>
               </span>
      case 'joint_liveliness':
        return <span>
                <span>Make the value of </span>
                <span style={{backgroundColor:'#BE33FF',color:'white',padding:4,borderRadius:5}}>
                  {this.getJointNameByIdx(objectiveData.objective.index)}
                </span>
                <span> have lifelike motion</span>
               </span>
      case 'base_link_position_liveliness':
      return <span>
              <span>Make the robot root </span>
              <span style={{backgroundColor:'#E73091',color:'white',padding:4,borderRadius:5}}>
                {this.props.config.fixed_frame}
              </span>
              <span> have lifelike motion</span>
             </span>
    }
  }

  getObjectiveEditor = (objectiveData) => {

  }

  getDefaultObjectives = (allObjectives) => {
    let defaultObjectives = [];
    allObjectives.forEach((objective,idx)=>{
      switch (objective.variant) {
        case 'joint_limits':
          defaultObjectives.push({objective:objective,index:idx});
          break;
        case 'nn_collision':
          defaultObjectives.push({objective:objective,index:idx});
          break;
        case 'env_collision':
          defaultObjectives.push({objective:objective,index:idx});
          break;
        case 'min_velocity':
          defaultObjectives.push({objective:objective,index:idx});
          break;
        case 'min_acceleration':
          defaultObjectives.push({objective:objective,index:idx});
          break;
        case 'min_jerk':
          defaultObjectives.push({objective:objective,index:idx});
          break;
        default:
          break;
      }
    })
    console.log(defaultObjectives)
    return defaultObjectives;
  }

  getDirectionObjectives = (allObjectives) => {
    let directionObjectives = [];
    allObjectives.forEach((objective,idx)=>{
      switch (objective.variant) {
        case 'ee_position_match':
          directionObjectives.push({objective:objective,index:idx});
          break;
        case 'ee_orientation_match':
          directionObjectives.push({objective:objective,index:idx});
          break;
        case 'ee_position_mirroring':
          directionObjectives.push({objective:objective,index:idx});
          break;
        case 'ee_orientation_mirroring':
          directionObjectives.push({objective:objective,index:idx});
          break;
        case 'ee_position_bounding':
          directionObjectives.push({objective:objective,index:idx});
          break;
        case 'ee_orientation_bounding':
          directionObjectives.push({objective:objective,index:idx});
          break;
        case 'joint_mirroring':
          directionObjectives.push({objective:objective,index:idx});
          break;
        case 'joint_match':
          directionObjectives.push({objective:objective,index:idx});
          break;
        default:
          break;
      }
    })
    return directionObjectives;
  }

  getLivelinessObjectives = (allObjectives) => {
    let livelinessObjectives = [];
    allObjectives.forEach((objective,idx)=>{
      switch (objective.variant) {
        case 'ee_position_liveliness':
          livelinessObjectives.push({objective:objective,index:idx});
          break;
        case 'ee_orientation_liveliness':
          livelinessObjectives.push({objective:objective,index:idx});
          break;
        case 'joint_liveliness':
          livelinessObjectives.push({objective:objective,index:idx});
          break;
        case 'base_link_position_liveliness':
          livelinessObjectives.push({objective:objective,index:idx});
          break;
        default:
          break;
      }
    })
    return livelinessObjectives;
  }

  render() {
    return (
      <Collapse defaultActiveKey={['1']} accordion={true}>
        <Panel header="Default" key="1">
          <List header={null} footer={null} bordered dataSource={this.getDefaultObjectives(this.props.config.objectives)}
                style={{overflow:'scroll',maxHeight:300,marginBottom:10}}
                renderItem={(item)=>(
                  <List.Item>{this.getObjectivePreview(item)}</List.Item>
                )}
          />
        </Panel>
        <Panel header="Directions" key="2">
          <List header={null} footer={null} bordered dataSource={this.getDirectionObjectives(this.props.config.objectives)}
                style={{overflow:'scroll',maxHeight:300,marginBottom:10}}
                renderItem={(item)=>(
                  <List.Item>{this.getObjectivePreview(item)}</List.Item>
                )}
          />
        </Panel>
        <Panel header="Liveliness" key="3">
          <List header={null} footer={null} bordered dataSource={this.getLivelinessObjectives(this.props.config.objectives)}
                style={{overflow:'scroll',maxHeight:300,marginBottom:10}}
                renderItem={(item)=>(
                  <List.Item>{this.getObjectivePreview(item)}</List.Item>
                )}
          />
        </Panel>
      </Collapse>
    )
  }

}

export default Behavior
