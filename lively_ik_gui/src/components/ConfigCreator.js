import React from 'react';
import { Steps, Divider, Button } from 'antd';
import Basics from './config/Basics';
import Collision from './config/Collision';
import Behavior from './config/Behavior';
const { Step } = Steps;

class ConfigCreator extends React.Component {

  state = {step:0};

  updateUrdf = (event) => {
    console.log(event.target.value);
    this.props.onUpdate({directive:'update',config:{urdf:event.target.value}})
  }

  updateFixedFrame = (fixedFrame) => {
    this.props.onUpdate({directive:'update',config:{fixed_frame:fixedFrame}})
  }

  updateJointNames = (jointNames) => {
    this.props.onUpdate({directive:'update',config:{joint_names:jointNames}})
  }

  updateEeFixedJoints = (eeFixedJoints) => {
    this.props.onUpdate({directive:'update',config:{ee_fixed_joints:eeFixedJoints}})
  }

  updateJointOrdering = (jointOrdering) => {
    this.props.onUpdate({directive:'update',config:{joint_ordering:jointOrdering}})
  }

  updateStates = (states) => {
    this.props.onUpdate({directive:'update',config:{states:states}})
  }

  updateStartingConfig = (startingConfig) => {
    this.props.onUpdate({directive:'update',config:{starting_config:startingConfig}})
  }

  updateRobotLinkRadius = (radius) => {
    this.props.onUpdate({directive:'update',config:{robot_link_radius:radius}})
  }

  updateStaticEnvironment = (staticEnvironment) => {
    this.props.onUpdate({directive:'update',config:{static_environment:staticEnvironment}})
  }

  beginPreprocess = () => {
    //this.props.socket.emit('app_process',{action:'preprocess'})
  }

  updateObjectives = (objectives) => {
    this.props.onUpdate({directive:'update',config:{objectives:objectives}})
  }

  updateGoals = (goals) => {
    this.props.onUpdate({directive:'update',config:{goals:goals}})
  }

  updateControlMode = (mode) => {
    this.props.onUpdate({directive:'update',config:{mode_control:mode}})
  }

  updateEnvironmentMode = (mode) => {
    this.props.onUpdate({directive:'update',config:{mode_environment:mode}})
  }

  updateBaseLinkMotionBounds = (bounds) => {
    this.props.onUpdate({directive:'update',config:{base_link_motion_bounds:bounds}})
  }

  updateDisplayedState = (displayedState) => {
    this.props.onUpdate({directive:'update',meta:{displayed_state:displayedState}})
  }

  updateToManual = () => {
    this.props.onUpdate({directive:'update',meta:{control:'manual'}})
  }

  updateToSolve = () => {
    this.props.onUpdate({directive:'update',meta:{control:'solve'}})
  }

  updateMeta = (meta) => {
    this.props.onUpdate({directive:'update',meta:meta})
  }

  canStep = (desired) => {
    switch (desired) {
      case 0:
        return (this.props.meta.valid_urdf && this.props.meta.valid_robot)
      case 1:
        return this.props.meta.valid_robot;
      case 2:
        return (this.props.meta.valid_config && this.props.meta.valid_solver)
      default:
        return false
    }
  }

  setStep = (step) => {
    if (step === 2) {
      this.updateToSolve()
    } else {
      this.updateToManual()
    }
    this.setState({step:step})
  }

  stepForward = () => {
    if (this.state.step+1 === 2) {
      this.updateToSolve()
    } else {
      this.updateToManual()
    }
    this.setState((state)=>({step:state.step+1}))
  }

  stepBackward = () => {
    if (this.state.step-1 === 2) {
      this.updateToSolve()
    } else {
      this.updateToManual()
    }
    this.setState((state)=>({step:state.step-1}))
  }

  getPage = () => {
    switch (this.state.step) {
      case 0:
        return (
          <Basics meta={this.props.meta}
                  config={this.props.config}
                  updateUrdf={(e)=>this.updateUrdf(e)}
                  updateFixedFrame={(e)=>this.updateFixedFrame(e)}
                  updateJointNames={(e)=>this.updateJointNames(e)}
                  updateJointOrdering={(e)=>this.updateJointOrdering(e)}
                  updateEeFixedJoints={(e)=>this.updateEeFixedJoints(e)}
                  updateControlMode={(e)=>this.updateControlMode(e)}
                  updateEnvironmentMode={(e)=>this.updateEnvironmentMode(e)}
                  updateMeta={(e)=>this.updateMeta(e)}
                  style={{height:'100%'}}
                  />
        );
      case 1:
        return (
          <Collision meta={this.props.meta}
                     config={this.props.config}
                     updateStates={(e)=>this.updateStates(e)}
                     updateStaticEnvironment={(e)=>this.updateStaticEnvironment(e)}
                     updateRobotLinkRadius={(e)=>this.updateRobotLinkRadius(e)}
                     updateMeta={(e)=>this.updateMeta(e)}
                     style={{height:'100%'}}
                  />
        );
      case 2:
        return (
          <Behavior meta={this.props.meta}
                    config={this.props.config}
                    updateObjectives={(e)=>this.updateObjectives(e)}
                    updateGoals={(e)=>this.updateGoals(e)}
                    style={{height:'100%'}}
                  />
        );
      default:
        return;
    }
  }

  render() {
    return (
      <div style={{margin:10,height:'100%'}}>
        <Steps current={this.state.step} size="large" type='navigation' onChange={(step)=>this.setStep(step)}>
          <Step title="Basics" description="Specify basic robot configuration" disabled={!this.canStep(0)}/>
          <Step title="Collision" description="Specify how the robot may collide" disabled={!this.canStep(1)}/>
          <Step title="Behavior" description="Specify how the robot behaves" disabled={!this.canStep(2)}/>
        </Steps>
        <div style={{margin:20,height:'100%'}}>
          {this.getPage()}
        </div>
      </div>
    )
  }

}

export default ConfigCreator
