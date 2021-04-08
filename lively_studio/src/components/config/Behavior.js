import React from 'react';
import { Tabs, List, Space, Tooltip, Button, Dropdown, Menu } from 'antd';
import { EditOutlined, CopyOutlined, DeleteOutlined } from '@ant-design/icons';
import { defaultObjectives, objectiveMeta } from '../../util/Default';
import { getObjectivePreview } from '../../util/Englishify';
import { BEHAVIOR_ATTRIBUTE_GROUPS,
         BEHAVIOR_ATTRIBUTE_GROUP_NAMES,
         PAIRED_OBJECTIVES,
         JOINT_OBJECTIVES,
         CARTESIAN_OBJECTIVES } from '../../util/Categories';
import {getObjectiveMarkers} from '../../util/Markers';
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

  getColor = (idx,selectionType) => {
    if (this.props.meta.selected === null || selectionType !== this.props.meta.selected.type) {
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

  createObjective = (variant) => {
    let objectives = [...this.props.config.objectives];
    let modes = [...this.props.config.modes];
    let goals = [...this.props.config.goals];

    let objective = {...defaultObjectives[variant]};
    let weight = objectiveMeta[variant].weight;
    let values = {...objectiveMeta[variant].goal};

    const isJointObjective = (JOINT_OBJECTIVES.indexOf(variant) >= 0);
    const isCartesianObjective = (CARTESIAN_OBJECTIVES.indexOf(variant) >= 0);
    const isPairedObjective = (PAIRED_OBJECTIVES.indexOf(variant) >= 0);

    if (isPairedObjective && isCartesianObjective && this.props.config.joint_names.length > 1) {
      objective.indices = [0, this.props.config.joint_names[0].length, 1, this.props.config.joint_names[1].length]
    } else if (isPairedObjective && isCartesianObjective) {
      objective.indices = [0, this.props.config.joint_names[0].length, 0, this.props.config.joint_names[0].length-1]
    } else if (isCartesianObjective) {
      objective.indices = [0, this.props.config.joint_names[0].length]
    } else if (isJointObjective && isPairedObjective) {
      objective.indices = [this.props.config.joint_ordering.length-1,this.props.config.joint_ordering.length-2]
    } else if (isJointObjective) {
      objective.indices = [this.props.config.joint_ordering.length-1]
    }

    if (variant === 'position_match') {
      let position = this.props.meta.joint_poses[objective.indices[0]][objective.indices[1]]['position']
      values.vector = [position.x,position.y,position.z];
    } else if (variant === 'orientation_match') {
      let quaternion = this.props.meta.joint_poses[objective.indices[0]][objective.indices[1]]['quaternion']
      values.quaternion = [quaternion.w,quaternion.x,quaternion.y,quaternion.z];
    } else if (variant === 'joint_match') {
      values.scalar = this.props.config.starting_config[1][objective.indices[0]]
    } else if (variant === 'base_link_position_match') {
      values.vector = this.props.config.starting_config[0];
    } else if (variant === 'position_boundng') {
      let position = this.props.meta.joint_poses[objective.indices[0]][objective.indices[1]]['position']
      values.pose = [[position.x,position.y,position.z],[1,0,0,0]]
    } else if (variant === 'distance_match') {
      let position1 = this.props.meta.joint_poses[objective.indices[0]][objective.indices[1]]['position'];
      let position2 = this.props.meta.joint_poses[objective.indices[2]][objective.indices[3]]['position'];
      values.scalar = Math.sqrt(Math.pow(position1.x-position2.x,2)+Math.pow(position1.y-position2.y,2)+Math.pow(position1.z-position2.z,2))
    }

    objectives.push(objective)
    modes.forEach((mode)=>{
      mode.weights.push(weight)
    })
    goals.forEach((goal)=>{
      goal.values.push(values)
    })

    let markers = getObjectiveMarkers({objective:objective,
                                       goal:values,
                                       tree:this.props.meta.robot_tree,
                                       poses:this.props.meta.joint_poses,
                                       jointNames:this.props.config.joint_names,
                                       jointOrdering:this.props.config.joint_ordering,
                                       eeFixedJoints:this.props.config.ee_fixed_joints,
                                       fixedFrame:this.props.config.fixed_frame
                                      });

    this.props.onUpdate({objectives:objectives,modes:modes,goals:goals},{selected:{idx:objectives.length-1,type:'objective'},gui_markers:markers})

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
    this.props.onUpdate({objectives:objectives,modes:modes,goals:goals},{selected:null,gui_markers:{}})
  }

  selectObjective = (idx) => {
    let markers = getObjectiveMarkers({objective:this.props.config.objectives[idx],
                                       goal:this.props.meta.target_goals[idx],
                                       tree:this.props.meta.robot_tree,
                                       poses:this.props.meta.joint_poses,
                                       jointNames:this.props.config.joint_names,
                                       jointOrdering:this.props.config.joint_ordering,
                                       eeFixedJoints:this.props.config.ee_fixed_joints,
                                       fixedFrame:this.props.config.fixed_frame
                                      });
    this.props.onUpdate({},{selected:{idx:idx,type:'objective'},
                            gui_markers:markers})
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
    let newModeWeights = this.props.config.objectives.map(objective=>objectiveMeta[objective.variant].weight);
    let mode = {name:this.getUnusedModeName(),weights:newModeWeights};
    modes.push(mode)
    this.props.onUpdate({modes:modes},{selected:{idx:modes.length-1,type:'mode'}})
  }

  addGoal = () => {
    let goals = [...this.props.config.goals];
    let newGoalValues = this.props.config.objectives.map(objective=>{
      switch(objective.variant) {
        case 'position_match':
          let pos = this.props.meta.joint_poses[objective.indices[0]][objective.indices[1]].position;
          return {vector:[pos.x,pos.y,pos.z]}
        case 'orientation_match':
          let ori = this.props.meta.joint_poses[objective.indices[0]][objective.indices[1]].quaternion;
          return {quaternion:[ori.w,ori.x,ori.y,ori.z]}
        case 'joint_match':
          let jointValue = this.props.meta.displayed_state[1][objective.indices[0]]
          return {scalar:jointValue}
        case 'base_link_position_match':
          let vec = this.props.meta.displayed_state[0];
          return {vector:vec}
        default:
          return {}
      }
    });
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
      <List.Item style={{backgroundColor:this.getColor(idx,'objective')}} extra={
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
        }
        onMouseEnter={()=>{
          let markers = getObjectiveMarkers({objective:objectiveData,
                                             goal:this.props.meta.target_goals[idx],
                                             poses:this.props.meta.joint_poses,
                                             tree:this.props.meta.robot_tree,
                                             jointNames:jointNames,
                                             jointOrdering:jointOrdering,
                                             eeFixedJoints:eeFixedJoints,
                                             fixedFrame:fixedFrame
                                            });
          this.props.onUpdate({},{gui_markers:markers})
        }}
        >
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
      <List.Item style={{backgroundColor:this.getColor(idx,'mode')}} extra={
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
      <List.Item style={{backgroundColor:this.getColor(idx,'goal')}} extra={
        <Space style={{width: 120}}>
          <Tooltip title='Edit'>
            <Button shape="circle" style={{marginLeft:5}} icon={<EditOutlined/>} onClick={()=>this.selectGoal(idx)}/>
          </Tooltip>
          <Tooltip title='Copy'>
            <Button shape="circle" style={{marginLeft:5}} icon={<CopyOutlined/>} onClick={()=>this.copyGoal(idx)}/>
          </Tooltip>
          <Tooltip title={this.props.config.goals[idx].name === 'default' ? 'Cannot be deleted' : 'Delete' }
                   color={this.props.config.goals[idx].name === 'default' ? null : '#ff4d4f'}>
            <Button shape="circle" style={{marginLeft:5}} icon={<DeleteOutlined/>} danger onClick={()=>this.deleteGoal(idx)}
                    disabled={this.props.config.goals[idx].name === 'default'}/>
          </Tooltip>
        </Space>
      }>
      <List.Item.Meta title={this.props.config.goals[idx].name === 'default' ? 'Default' : this.props.config.goals[idx].name}/>
    </List.Item>
    )
  }

  render() {
    return (
      <Tabs activeKey={this.state.overallTab}
            style={{height: '100%', width:'100%' }}
            onChange={(key)=>this.setOverallTab(key)}
            centered>
        <TabPane tab='Attributes' style={{height: '100%', width:'100%'}} key='attributes'>
          <Tabs activeKey={this.state.objectiveTab}
                style={{height: '100%', width:'100%' }}
                onChange={(key)=>this.setObjectiveTab(key)}
                tabPosition='left'>
            {BEHAVIOR_ATTRIBUTE_GROUPS.map((group,groupIdx)=>(
              <TabPane tab={BEHAVIOR_ATTRIBUTE_GROUP_NAMES[groupIdx]}
                       key={BEHAVIOR_ATTRIBUTE_GROUP_NAMES[groupIdx]}
                       style={{ height: '100%', width:'100%', overflowY:'scroll' }}>
                <List header={null}
                      footer={
                        <Dropdown trigger='click' overlay={
                          <Menu onClick={(e)=>this.createObjective(e.key)}>
                            {group.map((variant)=><Menu.Item key={variant}>{objectiveMeta[variant].name}</Menu.Item>)}
                          </Menu>
                        } placement="topLeft">
                          <Button>Add {BEHAVIOR_ATTRIBUTE_GROUP_NAMES[groupIdx]} Attribute</Button>
                        </Dropdown>
                      }
                      bordered
                      pagination={{
                        onChange: page => {},
                        pageSize: 7,
                      }}
                      style={{ maxHeight: '100%', width:'100%'}}
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
                dataSource={this.props.config.modes.map((mode,idx)=>idx)}
                renderItem={this.getModeListItem}
                pagination={{
                  onChange: page => {},
                  pageSize: 7,
                }}

          />
        </TabPane>
        <TabPane tab='Goals' style={{height: '100%', width:'100%' }} key='goals'>
          <List header={null} footer={<Button type="primary" onClick={this.addGoal}>Add Goal</Button>} bordered
                style={{ maxHeight: '100%', width:'100%', overflow:'scroll'}}
                dataSource={this.props.config.goals.map((goal,idx)=>idx)}
                renderItem={this.getGoalListItem}
                pagination={{
                  onChange: page => {},
                  pageSize: 7,
                }}

          />
        </TabPane>
      </Tabs>

    )
  }

}

export default Behavior
