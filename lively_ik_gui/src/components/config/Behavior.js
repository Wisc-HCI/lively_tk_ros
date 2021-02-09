import React from 'react';
import { Tabs, List, Space, Tooltip, Button } from 'antd';
import { EditOutlined, CopyOutlined, DeleteOutlined } from '@ant-design/icons';
import { defaultObjectives, defaultGoals } from '../../util/Default';
import { getObjectivePreview } from '../../util/Englishify';
const { TabPane } = Tabs;

const DEFAULT_OBJECTIVES = ['joint_limits','nn_collision','env_collision','min_velocity','min_acceleration','min_jerk'];
const DIRECTION_OBJECTIVES = ['ee_position_match','ee_orientation_match','ee_position_mirroring','ee_orientation_mirroring',
                              'ee_position_bounding','ee_orientation_bounding','joint_mirroring','joint_match'];
const LIVELINESS_OBJECTIVES = ['ee_position_liveliness','ee_orientation_liveliness','joint_liveliness','base_link_position_liveliness'];

class Behavior extends React.Component {

  getColor = (idx) => {
    if (this.props.meta.selected === null) {
      return 'white'
    } else if (idx === this.props.meta.selected.idx) {
      return '#E9E9E9'
    }
  }

  getEENameByIdx = (idx) => {
    return this.props.config.ee_fixed_joints[idx]
  }

  getJointNameByIdx = (idx) => {
    return this.props.config.joint_ordering[idx]
  }

  addObjective = (groupName) => {
    let objectives = [...this.props.config.objectives];
    let goals = [...this.props.config.goals];
    switch (groupName) {
      case 'Default':
        objectives.push(defaultObjectives.joint_limits)
        goals.forEach((goalMode)=>{
          goalMode.goals.push(defaultGoals.joint_limits)
        })
        break
      case 'Directions':
        objectives.push(defaultObjectives.ee_position_match)
        goals.forEach((goalMode)=>{
          goalMode.goals.push(defaultGoals.ee_position_match)
        })
        break
      case 'Liveliness':
        objectives.push(defaultObjectives.base_link_position_liveliness)
        goals.forEach((goalMode)=>{
          goalMode.goals.push(defaultGoals.base_link_position_liveliness)
        })
        break
      default:
        break
    }
    this.props.onUpdate({objectives:objectives,goals:goals},{selected:{idx:objectives.length-1,type:'objective'}})
  }

  deleteObjective = (idx) => {
    let objectives = [...this.props.config.objectives];
    let goals = [...this.props.config.goals];
    objectives.splice(idx,1);
    goals.forEach((goalMode)=>{
      goalMode.goals.splice(idx,1);
    })
    this.props.onUpdate({objectives:objectives,goals:goals},{selected:null})
  }

  selectObjective = (idx) => {
    this.props.onUpdate({},{selected:{idx:idx,type:'objective'}})
  }

  copyObjective = (idx) => {
    let objectives = [...this.props.config.objectives];
    let goals = [...this.props.config.goals];
    let copiedObjective = objectives[idx];
    objectives.splice(idx+1,0,copiedObjective)
    goals.forEach((goalMode)=>{
      let copiedGoal = goalMode.goals[idx];
      goalMode.goals.splice(idx+1,0,copiedGoal)
    })
    this.props.onUpdate({objectives:objectives,goals:goals},{selected:{idx:idx+1,type:'objective'}})
  }

  render() {
    const groups = [DEFAULT_OBJECTIVES,DIRECTION_OBJECTIVES,LIVELINESS_OBJECTIVES];
    const groupNames = ['Default','Directions','Liveliness'];
    return (
      <Tabs defaultActiveKey={[0]} tabPosition='left' style={{height: '100%', width:'100%' }}>
        {groups.map((group,groupIdx)=>(
          <TabPane tab={groupNames[groupIdx]} key={groupIdx} style={{ height: '100%', width:'100%' }}>
            <List header={null} footer={<Button type="primary" onClick={()=>this.addObjective(groupNames[groupIdx])}>Add {groupNames[groupIdx]} Behavior</Button>} bordered
                  style={{ maxHeight: '100%', width:'100%', overflow:'scroll'}}
                  dataSource={this.props.config.objectives.map((objective,idx)=>idx).filter((idx)=>group.indexOf(this.props.config.objectives[idx].variant)>=0)}
                  renderItem={(idx)=>(
                    <List.Item style={{backgroundColor:this.getColor(idx)}} extra={
                        <Space style={{width: 120}}>
                          <Tooltip title='Edit'>
                            <Button shape="circle" style={{marginLeft:5}} icon={<EditOutlined/>} onClick={()=>this.selectObjective(idx)}/>
                          </Tooltip>
                          <Tooltip title='Copy'>
                            <Button shape="circle" style={{marginLeft:5}} icon={<CopyOutlined/>} onClick={()=>this.copyObjective(idx)}/>
                          </Tooltip>
                          <Tooltip title='Delete' color='#ff4d4f'>
                            <Button shape="circle" style={{marginLeft:5}} icon={<DeleteOutlined/>} danger onClick={()=>this.deleteObjective(idx)}/>
                          </Tooltip>
                        </Space>
                      }>
                      <List.Item.Meta title={this.props.config.objectives[idx].tag} description={getObjectivePreview(this.props.config.objectives[idx], this.props.config.fixed_frame, this.props.config.ee_fixed_joints, this.props.config.joint_ordering)}/>
                    </List.Item>
                    )}
            />
          </TabPane>
        ))}
      </Tabs>
    )
  }

}

export default Behavior
