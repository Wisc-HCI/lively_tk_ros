import React from 'react';
import { Steps } from 'antd';
import Basics from './config/Basics';
import Collision from './config/Collision';
import Behavior from './config/Behavior';
const { Step } = Steps;

class ConfigCreator extends React.Component {

  state = {step:0};

  updateUrdf = (event) => {
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

  beginTraining = () => {
    this.props.onUpdate({directive:'process',type:'nn'})
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
    console.log(meta);
    this.props.onUpdate({directive:'update',meta:meta})
  }

  updateFromBehavior = (config,meta) => {
    this.props.onUpdate({directive:'update',meta:meta,config:config})
  }

  canStep = (desired) => {
    switch (desired) {
      case 0:
        return (this.props.meta.valid_urdf && this.props.meta.valid_robot)
      case 1:
        return this.props.meta.valid_robot;
      case 2:
        return (this.props.meta.valid_nn)
      default:
        return false
    }
  }

  setStep = (step) => {
    if (this.canStep(step)) {
      if (step === 2) {
        this.updateToSolve()
      } else {
        this.updateToManual()
      }
      this.setState({step:step})
    }

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
                  updateBaseLinkMotionBounds={(e)=>this.updateBaseLinkMotionBounds(e)}
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
                     updateShowLinkCollision={(b)=>this.updateMeta({show_link_collision:b})}
                     updateMeta={(e)=>this.updateMeta(e)}
                     beginTraining={()=>this.beginTraining()}
                     style={{height:'100%'}}
                  />
        );
      case 2:
        return (
          <Behavior meta={this.props.meta}
                    config={this.props.config}
                    onUpdate={(config,meta)=>this.updateFromBehavior(config,meta)}
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
        <Steps current={this.state.step} progressDot={true} onChange={(step)=>this.setStep(step)} style={{marginTop:20}}>
          <Step title="Basics" description="Specify basic robot configuration" disabled={!this.canStep(0)} onClick={this.setStep}/>
          <Step title="Collision" description="Specify how the robot may collide" disabled={!this.canStep(1)} onClick={this.setStep}/>
          <Step title="Behavior" description="Specify how the robot behaves" disabled={!this.canStep(2)} onClick={this.setStep}/>
        </Steps>
        <div style={{margin:20,height:'100%'}}>
          {this.getPage()}
        </div>
      </div>
    )
  }

}

export default ConfigCreator
