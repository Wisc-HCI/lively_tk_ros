import React from 'react';
import { Tabs, List, Space, Tooltip, Button } from 'antd';
import { EditOutlined, CopyOutlined, DeleteOutlined } from '@ant-design/icons';
import { defaultObjectives, defaultGoals, defaultWeights } from '../../util/Default';
import { getObjectivePreview } from '../../util/Englishify';
import { BEHAVIOR_ATTRIBUTE_GROUPS,
         BEHAVIOR_ATTRIBUTE_GROUP_NAMES } from '../../util/Categories';
const { TabPane } = Tabs;


class Behavior extends React.Component {

  constructor(props) {
    super(props);
    this.state = {objectiveTab:'Base',overallTab:'attributes'};
  }

  getUnusedModeName = () => {
    let name = 'New Mode'
    let nameIdx = 0;
    let existingNames = this.props.config.modes.map(mode=>mode.name);
    while (existingNames.indexOf(name) >= 0) {
      nameIdx += 1;
      name = `New Mode (${nameIdx})`
    }
    return name;
  }

  getUnusedGoalName = () => {
    let name = 'New Goal'
    let nameIdx = 0;
    let existingNames = this.props.config.goals.map(goal=>goal.name);
    while (existingNames.indexOf(name) >= 0) {
      nameIdx += 1;
      name = `New Goal (${nameIdx})`
    }
    return name;
  }

  getColor = (idx) => {
    if (this.props.meta.selected === null) {
      return 'white'
    } else if (idx === this.props.meta.selected.idx) {
      return '#E9E9E9'
    }
  }

  setObjectiveTab = (key) => {
    this.setState({objectiveTab:key})
  }

  setOverallTab = (key) => {
    this.setState({overallTab:key})
  }

  getEENameByIdx = (idx) => {
    return this.props.config.ee_fixed_joints[idx]
  }

  getJointNameByIdx = (idx) => {
    return this.props.config.joint_ordering[idx]
  }

  addObjective = (groupName) => {
    let objectives = [...this.props.config.objectives];
    let modes = [...this.props.config.modes];
    let goals = [...this.props.config.goals];
    switch (groupName) {
      case 'Base':
        objectives.push(defaultObjectives.joint_limits)
        modes.forEach((goalMode)=>{
          goalMode.weights.push(defaultWeights.joint_limits)
        })
        goals.forEach((goalSpec)=>{
          goalSpec.values.push(defaultGoals.joint_limits)
        })
        break
      case 'Directions':
        objectives.push(defaultObjectives.position_match)
        modes.forEach((goalMode)=>{
          goalMode.weights.push(defaultWeights.position_match)
        })
        goals.forEach((goalSpec)=>{
          let position = this.props.config.joint_poses[0][this.props.config.joint_poses[0].length-1]['position']
          goalSpec.values.push({vector:[position.x,position.y,position.z]})
        })
        break
      case 'Liveliness':
        objectives.push(defaultObjectives.base_link_position_liveliness)
        modes.forEach((goalMode)=>{
          goalMode.weights.push(defaultWeights.base_link_position_liveliness)
        })
        goals.forEach((goalSpec)=>{
          goalSpec.values.push(defaultGoals.base_link_position_liveliness)
        })
        break
      default:
        break
    }
    this.props.onUpdate({objectives:objectives,modes:modes,goals:goals},{selected:{idx:objectives.length-1,type:'objective'}})
  }

  deleteObjective = (idx) => {
    let objectives = [...this.props.config.objectives];
    let modes = [...this.props.config.modes];
    let goals = [...this.props.config.goals];
    objectives.splice(idx,1);
    modes.forEach((goalMode)=>{
      goalMode.weights.splice(idx,1);
    })
    goals.forEach((goalSpec)=>{
      goalSpec.values.splice(idx,1);
    })
    this.props.onUpdate({objectives:objectives,modes:modes,goals:goals},{selected:null})
  }

  selectObjective = (idx) => {
    this.props.onUpdate({},{selected:{idx:idx,type:'objective'}})
  }

  copyObjective = (idx) => {
    let objectives = [...this.props.config.objectives];
    let modes = [...this.props.config.modes];
    let goals = [...this.props.config.goals];
    let copiedObjective = objectives[idx];
    objectives.splice(idx+1,0,copiedObjective)
    modes.forEach((mode)=>{
      let copiedWeight = mode.weights[idx];
      mode.weights.splice(idx+1,0,copiedWeight)
    })
    goals.forEach((goal)=>{
      let copiedValue = goal.values[idx];
      goals.values.splice(idx+1,0,copiedValue)
    })
    this.props.onUpdate({objectives:objectives,modes:modes,goals:goals},{selected:{idx:idx+1,type:'objective'}})
  }

  addMode = () => {
    let modes = [...this.props.config.modes];
    let newModeWeights = this.props.config.objectives.map(objective=>defaultWeights[objective.variant]);
    let mode = {name:this.getUnusedModeName(),weights:newModeWeights};
    modes.push(mode)
    this.props.onUpdate({modes:modes},{selected:{idx:modes.length-1,type:'mode'}})
  }

  addGoal = () => {
    let goals = [...this.props.config.goals];
    let newGoalValues = this.props.config.objectives.map(objective=>defaultGoals[objective.variant]);
    let goal = {name:this.getUnusedGoalName(),values:newGoalValues};
    goals.push(goal)
    this.props.onUpdate({goals:goals},{selected:{idx:goals.length-1,type:'goal'}})
  }

  copyMode = (idx) => {
    let modes = [...this.props.config.modes];
    let copiedMode = JSON.parse(JSON.stringify(modes[idx]));
    if (copiedMode.name === 'default') {
      copiedMode.name = 'Copy of "Default"'
    } else {
      copiedMode.name = 'Copy of "'+copiedMode.name+'"'
    }
    modes.splice(idx+1,0,copiedMode)
    this.props.onUpdate({modes:modes},{selected:{idx:idx+1,type:'mode'}})
  }

  copyGoal = (idx) => {
    let goals = [...this.props.config.goals];
    let copiedGoal = JSON.parse(JSON.stringify(goals[idx]));
    if (copiedGoal.name === 'default') {
      copiedGoal.name = 'Copy of "Default"'
    } else {
      copiedGoal.name = 'Copy of "'+copiedGoal.name+'"'
    }
    goals.splice(idx+1,0,copiedGoal)
    this.props.onUpdate({goals:goals},{selected:{idx:idx+1,type:'goal'}})
  }

  selectMode = (idx) => {
    this.props.onUpdate({},{selected:{idx:idx,type:'mode'}})
  }

  selectGoal = (idx) => {
    this.props.onUpdate({},{selected:{idx:idx,type:'goal'}})
  }

  deleteMode = (idx) => {
    let modes = [...this.props.config.modes];
    modes.splice(idx,1);
    this.props.onUpdate({modes:modes},{selected:null})
  }

  deleteGoal = (idx) => {
    let goals = [...this.props.config.goals];
    goals.splice(idx,1);
    this.props.onUpdate({goals:goals},{selected:null})
  }

  getObjectiveListItem = (idx) => {
    const objectiveData = this.props.config.objectives[idx];
    const fixedFrame = this.props.config.fixed_frame;
    const eeFixedJoints = this.props.config.ee_fixed_joints;
    const jointOrdering = this.props.config.joint_ordering;
    const jointNames = this.props.config.joint_names;
    return (
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
        <List.Item.Meta title={this.props.config.objectives[idx].tag}
                        description={getObjectivePreview(objectiveData,
                                                         fixedFrame,
                                                         eeFixedJoints,
                                                         jointOrdering,
                                                         jointNames)}/>
      </List.Item>
    )
  }

  getModeListItem = (idx) => {
    return (
      <List.Item style={{backgroundColor:this.getColor(idx)}} extra={
        <Space style={{width: 120}}>
          <Tooltip title='Edit'>
            <Button shape="circle" style={{marginLeft:5}} icon={<EditOutlined/>} onClick={()=>this.selectMode(idx)}/>
          </Tooltip>
          <Tooltip title='Copy'>
            <Button shape="circle" style={{marginLeft:5}} icon={<CopyOutlined/>} onClick={()=>this.copyMode(idx)}/>
          </Tooltip>
          <Tooltip title={this.props.config.modes[idx].name === 'default' ? 'Cannot be deleted' : 'Delete' }
                   color={this.props.config.modes[idx].name === 'default' ? null : '#ff4d4f'}>
            <Button shape="circle" style={{marginLeft:5}} icon={<DeleteOutlined/>} danger onClick={()=>this.deleteMode(idx)}
                    disabled={this.props.config.modes[idx].name === 'default'}/>
          </Tooltip>
        </Space>
      }>
      <List.Item.Meta title={this.props.config.modes[idx].name === 'default' ? 'Default' : this.props.config.modes[idx].name}/>
    </List.Item>
    )
  }

  getGoalListItem = (idx) => {
    return (
      <List.Item style={{backgroundColor:this.getColor(idx)}} extra={
        <Space style={{width: 120}}>
          <Tooltip title='Edit'>
            <Button shape="circle" style={{marginLeft:5}} icon={<EditOutlined/>} onClick={()=>this.selectMode(idx)}/>
          </Tooltip>
          <Tooltip title='Copy'>
            <Button shape="circle" style={{marginLeft:5}} icon={<CopyOutlined/>} onClick={()=>this.copyMode(idx)}/>
          </Tooltip>
          <Tooltip title={this.props.config.modes[idx].name === 'default' ? 'Cannot be deleted' : 'Delete' }
                   color={this.props.config.modes[idx].name === 'default' ? null : '#ff4d4f'}>
            <Button shape="circle" style={{marginLeft:5}} icon={<DeleteOutlined/>} danger onClick={()=>this.deleteMode(idx)}
                    disabled={this.props.config.modes[idx].name === 'default'}/>
          </Tooltip>
        </Space>
      }>
      <List.Item.Meta title={this.props.config.modes[idx].name === 'default' ? 'Default' : this.props.config.modes[idx].name}/>
    </List.Item>
    )
  }

  render() {
    return (
      <Tabs activeKey={this.state.overallTab}
            style={{height: '100%', width:'100%' }}
            onChange={(key)=>this.setOverallTab(key)}
            centered>
        <TabPane tab='Attributes' style={{height: '100%', width:'100%' }} key='attributes'>
          <Tabs activeKey={this.state.objectiveTab}
                style={{height: '100%', width:'100%' }}
                onChange={(key)=>this.setObjectiveTab(key)}
                tabPosition='left'>
            {BEHAVIOR_ATTRIBUTE_GROUPS.map((group,groupIdx)=>(
              <TabPane tab={BEHAVIOR_ATTRIBUTE_GROUP_NAMES[groupIdx]}
                       key={BEHAVIOR_ATTRIBUTE_GROUP_NAMES[groupIdx]}
                       style={{ height: '100%', width:'100%' }}>
                <List header={null}
                      footer={<Button type="primary" onClick={()=>this.addObjective(BEHAVIOR_ATTRIBUTE_GROUP_NAMES[groupIdx])}>
                                Add {BEHAVIOR_ATTRIBUTE_GROUP_NAMES[groupIdx]} Attribute
                              </Button>}
                      bordered
                      style={{ maxHeight: '100%', width:'100%', overflow:'scroll'}}
                      dataSource={this.props.config.objectives.map((objective,idx)=>idx).filter((idx)=>group.indexOf(this.props.config.objectives[idx].variant)>=0)}
                      renderItem={this.getObjectiveListItem}
                />
              </TabPane>
            ))}
          </Tabs>
        </TabPane>
        <TabPane tab='Modes' style={{height: '100%', width:'100%' }} key='modes'>
          <List header={null} footer={<Button type="primary" onClick={this.addMode}>Add Mode</Button>} bordered
                style={{ maxHeight: '100%', width:'100%', overflow:'scroll'}}
                dataSource={this.props.config.modes.map((objective,idx)=>idx)}
                renderItem={this.getModeListItem}
          />
        </TabPane>
        <TabPane tab='Goals' style={{height: '100%', width:'100%' }} key='goals'>
          <List header={null} footer={<Button type="primary" onClick={this.addGoal}>Add Goal</Button>} bordered
                style={{ maxHeight: '100%', width:'100%', overflow:'scroll'}}
                dataSource={this.props.config.modes.map((objective,idx)=>idx)}
                renderItem={this.getGoalListItem}
          />
        </TabPane>
      </Tabs>

    )
  }

}

export default Behavior
