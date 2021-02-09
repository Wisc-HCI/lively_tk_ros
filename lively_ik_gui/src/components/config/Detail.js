import React from 'react';
import { Drawer } from 'antd';
import JointSpec from './JointSpec';
import ObjectiveSpec from './ObjectiveSpec';

class Detail extends React.Component {

  getTitle = () => {
    if (this.props.meta.selected) {
      switch (this.props.meta.selected.type) {
        case 'starting_config':
          return 'Specify Starting State'
        case 'collision_state':
          return 'Specify Collision State'
        case 'objective':
          return 'Specify Behavior'
        default:
          return 'NULL'
      }
    } else {
      return 'NULL'
    }

  }

  deselect = () => {
    this.props.onUpdate({directive:'update',meta:{selected:null}})
  }

  onUpdate = (values) => {
    if (this.props.meta.selected) {
      switch (this.props.meta.selected.type) {
        case 'starting_config':
          this.props.onUpdate({directive:'update',config:{starting_config:values},meta:{displayed_state:values}})
        case 'collision_state':
          let states = [...this.props.config.states];
          states[this.props.meta.selected.idx] = values;
          this.props.onUpdate({directive:'update',config:{states:states},meta:{displayed_state:values}})
        case 'objective':
          let objectives = [...this.props.config.objectives];
          let goals = [...this.props.config.goals];
          // Update the objective at the active index
          objectives[this.props.meta.selected.idx] = values.objective
          values.goals.forEach((goalMode,goalIdx)=>{
            goals[goalIdx].goals[this.props.meta.selected.idx] = goalMode
          })
          this.props.onUpdate({directive:'update',config:{objectives:objectives,goals:goals}})
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
          console.log(this.props.config.joint_limits);
          return <JointSpec joints={this.props.config.states[this.props.meta.selected.idx]}
                            names={this.props.config.joint_ordering}
                            limits={this.props.config.joint_limits}
                            onUpdate={this.onUpdate}/>
        case 'objective':
          return <ObjectiveSpec objective={this.props.config.objectives[this.props.meta.selected.idx]}
                                goals={this.props.config.goals.map(spec=>spec.goals[this.props.meta.selected.idx])}
                                eeFixedJoints={this.props.config.ee_fixed_joints}
                                fixedFrame={this.props.config.fixed_frame}
                                jointOrdering={this.props.config.joint_ordering}
                                modeNames={this.props.config.goals.map(spec=>spec.name)}
                                onUpdate={this.onUpdate}/>
        default:
          return 'NULL'
      }
    } else {
      return 'NULL'
    }
  }


  render() {
    return (
      <Drawer
        title={this.getTitle()}
        placement="right"
        closable={true}
        width='50%'
        onClose={this.deselect}
        visible={this.props.meta.selected !== null}
        getContainer={false}
        style={{ position: 'absolute' }}
      >
        {this.getContents()}
      </Drawer>
    )
  }

}

export default Detail
