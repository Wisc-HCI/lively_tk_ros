import React from 'react';
import _ from 'lodash';
import { Drawer, Button, Space } from 'antd';
import JointSpec from './JointSpec';
import ObjectiveSpec from './ObjectiveSpec';
import ModeSpec from './ModeSpec';
import GoalSpec from './GoalSpec';
const sidebars = ['starting_config','collision_state','objective','mode','goal'];

class Detail extends React.Component {

  constructor(props) {
    super(props);
    this.state = this.utilConstructInitial(this.props,true);
  }

  utilConstructInitial(props,copyIntialToCache=false) {
    let state = {initialData:null};
    if (props.meta.selected) {
      switch (props.meta.selected.type) {
        case 'objective':
          state.initialData = {cachedObjective:props.config.objectives[props.meta.selected.idx],
                           cachedModeWeights:props.config.modes.map((mode)=>mode.weights[props.meta.selected.idx]),
                           cachedGoalValues:props.config.goals.map((goal)=>goal.values[props.meta.selected.idx])}
          break
        case 'mode':
          state.initialData = props.config.modes[props.meta.selected.idx]
          break
        case 'goal':
          state.initialData = props.config.goals[props.meta.selected.idx]
          break
        default:
          state.initialData = null
          break
      }
    } else {
      state.initialData = null
    }
    if (copyIntialToCache) {
      state.cachedData = JSON.parse(JSON.stringify(state.initialData));
    }
    return state
  }

  componentDidUpdate(prevProps) {
    if (this.props.meta.selected !== prevProps.meta.selected) {
      this.setState(this.utilConstructInitial(this.props,true))
    } else {
      let candidateState = this.utilConstructInitial(this.props,false);
      let prevState = this.utilConstructInitial(prevProps,false);
      if (!_.isEqual(prevState.initialData, candidateState.initialData)) {
        console.log('component updated intial')
        this.setState(candidateState)
      }
    }
  }

  getTitle = () => {
    if (this.props.meta.selected) {
      switch (this.props.meta.selected.type) {
        case 'starting_config':
          return 'Specify Starting State'
        case 'collision_state':
          return 'Specify Collision State'
        case 'objective':
          return 'Specify Behavior Attribute'
        case 'mode':
          return 'Specify Mode'
        case 'goal':
          return 'Specify Goal Values'
        default:
          return ''
      }
    } else {
      return ''
    }

  }

  getIsSame = () => {
    if (this.props.meta.selected) {
      return _.isEqual(this.state.initialData, this.state.cachedData);
    }
    return true
  }

  deselect = () => {
    let meta = {selected:null};
    if (this.props.meta.selected.type === 'mode') {
      meta.active_mode = this.props.meta.selected.idx
    } else if (this.props.meta.selected.type === 'goal') {
      meta.active_goals = this.props.meta.selected.idx
    }
    this.props.onUpdate({directive:'update',meta:meta})
  }

  onSave = () => {
    if (this.props.meta.selected) {
      let objectives = [...this.props.config.objectives];
      let modes = [...this.props.config.modes];
      let goals = [...this.props.config.goals];
      switch (this.props.meta.selected.type) {
        case 'mode':
          modes[this.props.meta.selected.idx] = this.state.cachedData;
          this.props.onUpdate({directive:'update',config:{modes:modes}})
          break
        case 'goal':
          goals[this.props.meta.selected.idx] = this.state.cachedData;
          this.props.onUpdate({directive:'update',config:{goals:goals}})
          break
        case 'objective':
          objectives[this.props.meta.selected.idx] = this.state.cachedData.cachedObjective;
          modes = modes.map((modeInfo,idx)=>{
            let newModeInfo = {...modeInfo};
            newModeInfo.weights[this.props.meta.selected.idx] = this.state.cachedData.cachedModeWeights[idx]
            return newModeInfo;
          })
          goals = goals.map((goalInfo,idx)=>{
            let newGoalInfo = {...goalInfo};
            newGoalInfo.values[this.props.meta.selected.idx] = this.state.cachedData.cachedGoalValues[idx]
            return newGoalInfo;
          })
          this.props.onUpdate({directive:'update',config:{objectives:objectives,goals:goals,modes:modes}})
          break
        default:
      }
    }
  }

  onUpdate = (values) => {
    if (this.props.meta.selected) {
      let states = [...this.props.config.states];
      switch (this.props.meta.selected.type) {
        case 'starting_config':
          this.props.onUpdate({directive:'update',config:{starting_config:values},meta:{displayed_state:values}})
          break;
        case 'collision_state':
          states[this.props.meta.selected.idx] = values;
          this.props.onUpdate({directive:'update',config:{states:states},meta:{displayed_state:values}})
          break;
        case 'objective':
          if (values.cachedData !== undefined) {
            this.setState({cachedData:values.cachedData},console.log(this.getIsSame()))
          }
          break;
        case 'mode':
          if (values.targetModeWeights !== undefined) {
            this.props.onUpdate({directive:'update',meta:{target_weights:values.targetModeWeights}})
          }
          if (values.cachedData !== undefined) {
            this.setState({cachedData:values.cachedData})
          }
          break;
        case 'goal':
          if (values.targetGoalValues !== undefined) {
            this.props.onUpdate({directive:'update',meta:{target_goals:values.targetGoalValues}})
          }
          if (values.cachedData !== undefined) {
            this.setState({cachedData:values.cachedData})
          }
          break;
        default:

      }
    }
  }

  getContents = () => {
    if (this.props.meta.selected) {
      switch (this.props.meta.selected.type) {
        case 'starting_config':
          return <JointSpec state={this.props.config.starting_config}
                            names={this.props.config.joint_ordering}
                            baseLimits={this.props.config.base_link_motion_bounds}
                            jointLimits={this.props.config.joint_limits}
                            onUpdate={this.onUpdate}/>
        case 'collision_state':
          return <JointSpec state={this.props.config.states[this.props.meta.selected.idx]}
                            names={this.props.config.joint_ordering}
                            baseLimits={this.props.config.base_link_motion_bounds}
                            jointLimits={this.props.config.joint_limits}
                            onUpdate={this.onUpdate}/>
        case 'objective':
          return <ObjectiveSpec objective={this.props.config.objectives[this.props.meta.selected.idx]}
                                modeWeights={this.props.config.modes.map(mode=>mode.weights[this.props.meta.selected.idx])}
                                goalValues={this.props.config.goals.map(goal=>goal.values[this.props.meta.selected.idx])}
                                jointLimits={this.props.config.joint_limits}
                                eeFixedJoints={this.props.config.ee_fixed_joints}
                                jointPoses={this.props.meta.joint_poses}
                                fixedFrame={this.props.config.fixed_frame}
                                jointOrdering={this.props.config.joint_ordering}
                                jointNames={this.props.config.joint_names}
                                modeNames={this.props.config.modes.map(mode=>mode.name)}
                                goalNames={this.props.config.goals.map(goal=>goal.name)}
                                displayedState={this.props.meta.displayed_state}
                                startingConfig={this.props.config.starting_config}
                                onUpdate={this.onUpdate}/>
        case 'mode':
          return <ModeSpec modeInfo={this.props.config.modes[this.props.meta.selected.idx]}
                           modeNames={this.props.config.modes.map(mode=>mode.name)}
                           objectives={this.props.config.objectives}
                           onUpdate={this.onUpdate}/>
       case 'goal':
         return <GoalSpec goalInfo={this.props.config.goals[this.props.meta.selected.idx]}
                          goalNames={this.props.config.goals.map(goal=>goal.name)}
                          objectives={this.props.config.objectives}
                          jointLimits={this.props.config.joint_limits}
                          displayedState={this.props.meta.displayed_state}
                          onUpdate={this.onUpdate}/>
        default:
          return ''
      }
    } else {
      return ''
    }
  }

  setClosable = (value) => {
    this.setState({closable:value})
  }

  render() {
    return (

      <Drawer
        title={this.getTitle()}
        footer={this.getIsSame() ? null : <Space>
                                                <Button block ghost danger onClick={()=>this.deselect()}>Cancel</Button>
                                                <Button block ghost type='primary' onClick={()=>this.onSave()}>Save</Button>
                                           </Space>}
        placement="right"
        closable={this.getIsSame()}
        maskClosable={this.getIsSame()}
        width='50%'
        onClose={this.deselect}
        visible={this.props.meta.selected != null && sidebars.indexOf(this.props.meta.selected.type)>=0}
        getContainer={false}
        style={{ position: 'absolute' }}
      >
        {this.getContents()}
      </Drawer>
    )
  }

}

export default Detail
