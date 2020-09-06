import React from 'react';
import { Button, Descriptions, Card, Select, InputNumber } from 'antd';
import {PlusOutlined,DeleteOutlined} from '@ant-design/icons';
const { Option } = Select;

class Objectives extends React.Component {

  removeObjective = (idx) => {
    let objectives = [...this.props.objectives];
    objectives.splice(idx,1)
    this.props.updateObjectives(objectives);
  }

  createObjective = () => {
    let objectives = [...this.props.objectives];
    objectives.push({type:'position',index:1,weight: 50.0,gradient:'forward_ad'});
    this.props.updateObjectives(objectives);
  }

  onChangeType = (futureType,idx) => {
    let currentObjective = this.props.objectives[idx];
    let newObjective;
    if (futureType === 'position' || futureType === 'rotation' || futureType === 'dc') {
      let {position, index = 1, weight, gradient} = currentObjective;
      newObjective = {type:futureType,position:position,index:index,weight:weight,gradient:gradient};
    } else if (futureType === 'positional_noise' || futureType === 'rotational_noise' || futureType === 'dc_noise') {
      let {position, index = 1, weight, gradient, scale = 1.0, frequency = 5.0} = currentObjective;
      newObjective = {type:futureType,position:position,index:index,weight:weight,gradient:gradient,scale:scale,frequency:frequency};
    } else if (futureType === 'min_jt_vel' || futureType === 'min_jt_accel' || futureType === 'min_jt_jerk' || futureType === 'joint_limit' || futureType === 'collision_nn') {
      let {position, weight, gradient} = currentObjective;
      newObjective = {type:futureType,position:position,weight:weight,gradient:gradient};
    } else if (futureType === 'x_position_match' || futureType === 'y_position_match' || futureType === 'z_position_match') {
      let {position, index_1 = 1, index_2 = 1, delta = 0.0, weight, gradient} = currentObjective;
      newObjective = {type:futureType,position:position,index_1:index_1,index_2:index_2,delta:delta,weight:weight,gradient:gradient};
    } else if (futureType === 'rotation_match' || futureType === 'joint_match') {
      let {position, index_1 = 1, index_2 = 1, weight, gradient} = currentObjective;
      newObjective = {type:futureType,position:position,index_1:index_1,index_2:index_2,weight:weight,gradient:gradient};
    }
    let objectives = [...this.props.objectives];
    objectives[idx] = newObjective;
    this.props.updateObjectives(objectives)
  }

  onChangeIndex = (futureIndex,idx) => {
    let objective = this.props.objectives[idx];
    objective.index = futureIndex;
    let objectives = [...this.props.objectives];
    objectives[idx] = objective;
    this.props.updateObjectives(objectives)
  }

  onChangeIndexOne = (futureIndex,idx) => {
    let objective = this.props.objectives[idx];
    objective.index_1 = futureIndex;
    let objectives = [...this.props.objectives];
    objectives[idx] = objective;
    this.props.updateObjectives(objectives)
  }

  onChangeIndexTwo = (futureIndex,idx) => {
    let objective = this.props.objectives[idx];
    objective.index_2 = futureIndex;
    let objectives = [...this.props.objectives];
    objectives[idx] = objective;
    this.props.updateObjectives(objectives)
  }

  onChangeWeight = (futureWeight,idx) => {
    let objective = this.props.objectives[idx];
    objective.weight = futureWeight;
    let objectives = [...this.props.objectives];
    objectives[idx] = objective;
    this.props.updateObjectives(objectives)
  }

  onChangeGradient = (futureGradient,idx) => {
    let objective = this.props.objectives[idx];
    objective.gradient = futureGradient;
    let objectives = [...this.props.objectives];
    objectives[idx] = objective;
    this.props.updateObjectives(objectives)
  }

  onChangeFrequency = (futureFrequency,idx) => {
    let objective = this.props.objectives[idx];
    objective.frequency = futureFrequency;
    let objectives = [...this.props.objectives];
    objectives[idx] = objective;
    this.props.updateObjectives(objectives)
  }

  onChangeScale = (futureScale,idx) => {
    let objective = this.props.objectives[idx];
    objective.scale = futureScale;
    let objectives = [...this.props.objectives];
    objectives[idx] = objective;
    this.props.updateObjectives(objectives)
  }

  getObjective = (objective,idx) => {
    return (
      <Card.Grid style={{width:'100%'}}>
        <Descriptions extra={<Button danger icon={<DeleteOutlined />}/>}>
          <Descriptions.Item label="Type">
            <Select style={{width:250}} onChange={(e)=>this.onChangeType(e,idx)} value={objective.type}>
              <Option value="position">Position</Option>
              <Option value="rotation">Rotation</Option>
              <Option value="positional_noise">Positional Noise</Option>
              <Option value="rotational_noise">Rotational Noise</Option>
              <Option value="dc">Direct Joint Control</Option>
              <Option value="dc_noise">Joint Noise</Option>
              <Option value="min_jt_vel">Joint Velocity Minimization</Option>
              <Option value="min_jt_accel">Joint Acceleration Minimization</Option>
              <Option value="min_jt_jerk">Joint Jerk Minimization</Option>
              <Option value="joint_limit">Joint Limits</Option>
              <Option value="collision_nn">Collision Avoidance</Option>
              <Option value="x_position_match">Match X Positions</Option>
              <Option value="y_position_match">Match Y Positions</Option>
              <Option value="z_position_match">Match Z Positions</Option>
              <Option value="rotation_match">Match Rotations</Option>
              <Option value="joint_match">Match Joints</Option>
            </Select>
          </Descriptions.Item>
          {objective.index ? (
            <Descriptions.Item label="Index">
              {(objective.type === 'dc' || objective.type === 'dc_noise' || objective.type === 'dc_noise') ? (
                <Select style={{width:250}} onChange={(e)=>this.onChangeIndex(e,idx)} value={objective.index}>
                  {this.props.jointOrdering.map((joint,jointIdx)=>{
                    return <Option value={jointIdx+1}>{joint}</Option>
                  })}
                </Select>
              ) : (
                <Select style={{width:250}} onChange={(e)=>this.onChangeIndex(e,idx)} value={objective.index}>
                  {this.props.eeFixedJoints.map((joint,jointIdx)=>{
                    return <Option value={jointIdx+1}>{joint}</Option>
                  })}
                </Select>
              )}
            </Descriptions.Item>
          ) : (
            <></>
          )}
          {objective.index_1 ? (
            <Descriptions.Item label="Index 1">
              {(objective.type === 'joint_match') ? (
                <Select style={{width:200}} onChange={(e)=>this.onChangeIndexOne(e,idx)} value={objective.index_1}>
                  {this.props.jointOrdering.map((joint,jointIdx)=>{
                    return <Option value={jointIdx+1}>{joint}</Option>
                  })}
                </Select>
              ) : (
                <Select style={{width:200}} onChange={(e)=>this.onChangeIndexOne(e,idx)} value={objective.index_1}>
                  {this.props.eeFixedJoints.map((joint,jointIdx)=>{
                    return <Option value={jointIdx+1}>{joint}</Option>
                  })}
                </Select>
              )}
            </Descriptions.Item>
          ) : (
            <></>
          )}
          {objective.index_2 ? (
            <Descriptions.Item label="Index 2">
              {(objective.type === 'joint_match') ? (
                <Select style={{width:200}} onChange={(e)=>this.onChangeIndexTwo(e,idx)} value={objective.index_2}>
                  {this.props.jointOrdering.map((joint,jointIdx)=>{
                    return <Option value={jointIdx+1}>{joint}</Option>
                  })}
                </Select>
              ) : (
                <Select style={{width:200}} onChange={(e)=>this.onChangeIndexTwo(e,idx)} value={objective.index_2}>
                  {this.props.eeFixedJoints.map((joint,jointIdx)=>{
                    return <Option value={jointIdx+1}>{joint}</Option>
                  })}
                </Select>
              )}
            </Descriptions.Item>
          ) : (
            <></>
          )}
          <Descriptions.Item label="Weight">
            <InputNumber value={objective.weight} min={0} max={50} step={0.1} onChange={(e)=>this.onChangeWeight(e,idx)} />
          </Descriptions.Item>
          <Descriptions.Item label="Gradient">
            <Select style={{width:200}} onChange={(e)=>this.onChangeGradient(e,idx)} value={objective.gradient}>
              <Option value='forward_ad'>Forward Diff</Option>
              <Option value='reverse_ad'>Reverse Diff</Option>
              <Option value='finite_diff'>Finite Diff</Option>
            </Select>
          </Descriptions.Item>
          {objective.frequency ? (
            <Descriptions.Item label="Frequency">
              <InputNumber value={objective.frequency} min={0} max={50} step={0.1} onChange={(e)=>this.onChangeFrequency(e,idx)} />
            </Descriptions.Item>
          ) : (
            <></>
          )}
          {objective.scale ? (
            <Descriptions.Item label="Scale">
              <InputNumber value={objective.scale} min={0} max={50} step={0.1} onChange={(e)=>this.onChangeScale(e,idx)} />
            </Descriptions.Item>
          ) : (
            <></>
          )}
        </Descriptions>
      </Card.Grid>
    );
  }

  render() {
    return (
      <>
        <h5 style={{backgroundColor:'#e8e8e8', borderRadius:3, padding:10}}>
          Specify objectives to use with LivelyIK
        </h5>
        <Card title="Objectives" extra={<Button onClick={()=>this.createObjective()} icon={<PlusOutlined/>}></Button>}>
          {this.props.objectives.map(this.getObjective)}
        </Card>
      </>
    )
  }

}

export default Objectives
