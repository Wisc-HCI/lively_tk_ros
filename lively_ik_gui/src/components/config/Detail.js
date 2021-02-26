import React from 'react';
import { Drawer } from 'antd';
import JointSpec from './JointSpec';
import ObjectiveSpec from './ObjectiveSpec';
import ModeSpec from './ModeSpec';
import GoalSpec from './GoalSpec';
const sidebars = ['starting_config','collision_state','objective','mode'];

class Detail extends React.Component {

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
        default:
          return ''
      }
    } else {
      return ''
    }

  }

  deselect = () => {
    let meta = {selected:null};
    let config = {};

    if (this.props.meta.selected.type === 'mode') {
      let modes = [...this.props.config.modes]
      this.props.meta.target_weights.forEach((weight,idx)=>{
        modes[this.props.meta.selected.idx].weights[idx] = weight
      })
      config.modes = modes
    }
    this.props.onUpdate({directive:'update',meta:meta,config:config})
  }

  onUpdate = (values) => {
    let states = [...this.props.config.states];
    let objectives = [...this.props.config.objectives];
    let modes = [...this.props.config.modes];
    let goals = [...this.props.config.goals];
    if (this.props.meta.selected) {
      switch (this.props.meta.selected.type) {
        case 'starting_config':
          this.props.onUpdate({directive:'update',config:{starting_config:values},meta:{displayed_state:values}})
          break;
        case 'collision_state':
          states[this.props.meta.selected.idx] = values;
          this.props.onUpdate({directive:'update',config:{states:states},meta:{displayed_state:values}})
          break;
        case 'objective':
          if (values.objective !== undefined) {
            // Update the objective at the active index
            objectives[this.props.meta.selected.idx] = values.objective
          }
          if (values.modeWeights !== undefined) {
            values.modeWeights.forEach((modeWeight,modeIdx)=>{
              modes[modeIdx].weights[this.props.meta.selected.idx] = modeWeight
            })
          }
          if (values.goalValues !== undefined) {
            values.goalValues.forEach((goalValue,goalIdx)=>{
              goals[goalIdx].values[this.props.meta.selected.idx] = goalValue
            })
          }
          this.props.onUpdate({directive:'update',config:{objectives:objectives,modes:modes,goals:goals}})
          break;
        case 'mode':
          let configChanges = {};
          let metaChanges = {};
          if (values.name !== undefined) {
            modes[this.props.meta.selected.idx].name = values.name
            configChanges.modes = modes
          }
          if (values.targetWeights !== undefined) {
            metaChanges.target_weights = values.targetWeights;
          }
          this.props.onUpdate({directive:'update',config:configChanges,meta:metaChanges})
          break;
        default:

      }
    }
  }

  getContents = () => {
    if (this.props.meta.selected) {
      switch (this.props.meta.selected.type) {
        case 'starting_config':
          return <JointSpec joints={this.props.config.starting_config}
                            names={this.props.config.joint_ordering}
                            limits={this.props.config.joint_limits}
                            onUpdate={this.onUpdate}/>
        case 'collision_state':
          return <JointSpec joints={this.props.config.states[this.props.meta.selected.idx]}
                            names={this.props.config.joint_ordering}
                            limits={this.props.config.joint_limits}
                            onUpdate={this.onUpdate}/>
        case 'objective':
          return <ObjectiveSpec objective={this.props.config.objectives[this.props.meta.selected.idx]}
                                modeWeights={this.props.config.modes.map(mode=>mode.weights[this.props.meta.selected.idx])}
                                goalValues={this.props.config.goals.map(goal=>goal.values[this.props.meta.selected.idx])}
                                eeFixedJoints={this.props.config.ee_fixed_joints}
                                jointPoses={this.props.meta.joint_poses}
                                fixedFrame={this.props.config.fixed_frame}
                                jointOrdering={this.props.config.joint_ordering}
                                jointNames={this.props.config.joint_names}
                                modeNames={this.props.config.modes.map(mode=>mode.name)}
                                goalNames={this.props.config.goals.map(goal=>goal.name)}
                                onUpdate={this.onUpdate}/>
        case 'mode':
          return <ModeSpec modeName={this.props.config.modes[this.props.meta.selected.idx].name}
                           modeNames={this.props.config.modes.map(mode=>mode.name)}
                           targetWeights={this.props.meta.target_weights}
                           weights={this.props.config.modes[this.props.meta.selected.idx].weights}
                           names={this.props.config.objectives.map(obj=>obj.tag)}
                           variants={this.props.config.objectives.map(obj=>obj.variant)}
                           setClosable={this.setClosable}
                           onUpdate={this.onUpdate}/>
       case 'goal':
         return <GoalSpec goalName={this.props.config.goals[this.props.meta.selected.idx].name}
                          goalValues={this.props.config.goals[this.props.meta.selected.idx].values}
                          goalNames={this.props.config.goals.map(goal=>goal.name)}
                          objectives={this.props.config.objectives}
                          setClosable={this.setClosable}
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
        placement="right"
        closable={true}
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
