import React from 'react';
import { Steps, Divider, Button } from 'antd';
import Basic from './config/Basic';
import Joints from './config/Joints';
import Initial from './config/Initial';
import Collision from './config/Collision';
import Objects from './config/Objects';
import Objectives from './config/Objectives';
import Misc from './config/Misc';
import Preprocessing from './config/Preprocessing';
const { Step } = Steps;

class ConfigCreator extends React.Component {

  reset = () => {
    //this.props.socket.emit('app_update',{action:'config_update',...clearedState})
  }

  stepForward = () => {
    this.props.onSwitchStep(this.props.step+1)
  }

  stepBackward = () => {
    this.props.onSwitchStep(this.props.step-1)
  }

  updateFixedFrame = (fixedFrame) => {
    //this.props.socket.emit('app_update',{action:'config_update',config:{fixedFrame:fixedFrame}})
  }

  updateUrdf = (event) => {
    let urdf = event.target.value;
    if (urdf === '') {
      urdf = null;
    }
    //this.props.socket.emit('app_update',{action:'config_update',config:{urdf:urdf}})
  }

  updateJointOrdering = (jointOrdering) => {
    //this.props.socket.emit('app_update',{action:'config_update',config:{jointOrdering:jointOrdering}})
  }

  updateJointNames = (jointNames) => {
    //this.props.socket.emit('app_update',{action:'config_update',config:{jointNames:jointNames}})
  }

  updateEeFixedJoints = (eeFixedJoints) => {
    //this.props.socket.emit('app_update',{action:'config_update',config:{eeFixedJoints:eeFixedJoints}})
  }

  updateStartingConfig = (startingConfig) => {
    //this.props.socket.emit('app_update',{action:'config_update',app:{displayedState:startingConfig},config:{startingConfig:startingConfig}})
  }

  updateDisplayedState = (displayedState) => {
    //this.props.socket.emit('app_update',{action:'config_update',app:{displayedState:displayedState},config:{}})
  }

  updateSampleStates = (sampleStates) => {
    //this.props.socket.emit('app_update',{action:'config_update',config:{sampleStates:sampleStates}})
  }

  updateTrainingStates = (trainingStates) => {
    //this.props.socket.emit('app_update',{action:'config_update',config:{trainingStates:trainingStates}})
  }

  updateProblemStates = (problemStates) => {
    //this.props.socket.emit('app_update',{action:'config_update',config:{problemStates:problemStates}})
  }

  updateObjectives = (objectives) => {
    //this.props.socket.emit('app_update',{action:'config_update',config:{objectives:objectives}})
  }

  updateBoxes = (boxes) => {
    //this.props.socket.emit('app_update',{action:'config_update',config:{boxes:boxes}})
  }

  updateSpheres = (spheres) => {
    //this.props.socket.emit('app_update',{action:'config_update',config:{spheres:spheres}})
  }

  updateEllipsoids = (ellipsoids) => {
    //this.props.socket.emit('app_update',{action:'config_update',config:{ellipsoids:ellipsoids}})
  }

  updateCapsules = (capsules) => {
    //this.props.socket.emit('app_update',{action:'config_update',config:{capsules:capsules}})
  }

  updateCylinders = (cylinders) => {
    //this.props.socket.emit('app_update',{action:'config_update',config:{cylinders:cylinders}})
  }

  updateMode = (mode) => {
    //this.props.socket.emit('app_update',{action:'config_update',config:{mode:mode}})
  }

  updateRobotLinkRadius = (radius) => {
    //this.props.socket.emit('app_update',{action:'config_update',config:{robotLinkRadius:radius}})
  }

  updateFixedFrameNoiseScale = (scale) => {
    //this.props.socket.emit('app_update',{action:'config_update',config:{fixedFrameNoiseScale:scale}})
  }

  updateFixedFrameNoiseFrequency = (frequency) => {
    //this.props.socket.emit('app_update',{action:'config_update',config:{fixedFrameNoiseFrequency:frequency}})
  }

  beginPreprocess = () => {
    //this.props.socket.emit('app_process',{action:'preprocess'})
  }

  getPage = () => {
    switch (this.props.step) {
      case 0:
        return (
          <Basic urdf={this.props.config.urdf}
                 robotName=''
                 fixedFrame={this.props.config.fixed_frame}
                 allLinks={[]}
                 updateRobotName={(e)=>this.updateRobotName(e)}
                 updateFixedFrame={(e)=>this.updateFixedFrame(e)}
                 updateUrdf={(e)=>this.updateUrdf(e)}/>
        );
      case 1:
        return (
          <Joints
                  jointOrdering={this.props.config.joint_ordering}
                  jointNames={this.props.config.joint_names}
                  eeFixedJoints={[]}
                  allFixedJoints={[]}
                  allDynamicJoints={[]}
                  updateJointOrdering={(list)=>this.updateJointOrdering(list)}
                  updateJointNames={(names)=>this.updateJointNames(names)}
                  updateEeFixedJoints={(names)=>this.updateEeFixedJoints(names)}/>
        );
      case 2:
        return (
          <Initial jointOrdering={this.props.config.joint_ordering}
                   jointLimits={this.props.config.joint_limits}
                   startingConfig={this.props.config.starting_config}
                   updateStartingConfig={(state)=>this.updateStartingConfig(state)}/>
        );
      case 3:
        return (
          <Collision jointOrdering={this.props.config.joint_ordering}
                     jointLimits={this.props.config.joint_limits}
                     displayedState={this.state.config.starting_config}
                     sampleStates={[]}
                     trainingStates={[]}
                     problemStates={[]}
                     updateDisplayedState={(state)=>this.updateDisplayedState(state)}
                     updateSampleStates={(states)=>this.updateSampleStates(states)}
                     updateTrainingStates={(states)=>this.updateTrainingStates(states)}
                     updateProblemStates={(states)=>this.updateProblemStates(states)}/>
        );
      case 4:
        return (
          <Objects jointOrdering={this.props.config.joint_ordering}
                   boxes={[]}
                   spheres={[]}
                   ellipsoids={[]}
                   capsules={[]}
                   cylinders={[]}
                   updateBoxes={(objects)=>this.updateBoxes(objects)}
                   updateSpheres={(objects)=>this.updateSpheres(objects)}
                   updateEllipsoids={(objects)=>this.updateEllipsoids(objects)}
                   updateCapsules={(objects)=>this.updateCapsules(objects)}
                   updateCylinders={(objects)=>this.updateCylinders(objects)}/>
        );
      case 5:
        return (
          <Objectives objectives={[]}
                      eeFixedJoints={[]}
                      jointOrdering={this.props.config.joint_ordering}
                      updateObjectives={(objectives)=>this.updateObjectives(objectives)}/>
        );
      case 6:
        return (
          <Misc mode={this.props.config.mode}
                robotLinkRadius={this.props.config.robot_link_radius}
                fixedFrameNoiseScale={0}
                fixedFrameNoiseFrequency={0}
                updateMode={(mode)=>this.updateMode(mode)}
                updateRobotLinkRadius={(radius)=>this.updateRobotLinkRadius(radius)}
                updateFixedFrameNoiseScale={(scale)=>this.updateFixedFrameNoiseScale(scale)}
                updateFixedFrameNoiseFrequency={(freq)=>this.updateFixedFrameNoiseFrequency(freq)}/>
        )
      case 7:
        return (
          <Preprocessing preprocessingState={this.state.app.preprocessingState}
                         beginPreprocess={()=>this.beginPreprocess()}/>
        )
      default:
        return;
    }
  }

  render() {
    return (
      <div style={{margin:10}}>
        <Steps current={this.props.step} size="large">
          <Step title="Basics" description="Specify basic robot configuration"/>
          <Step title="Collision" description="Specify how the robot may collide"/>
          <Step title="Behavior" description="Specify how the robot behaves"/>
        </Steps>
        <div style={{margin:20}}>
          {this.getPage()}
        </div>
        <Divider/>
        <div style={{display:'flex',justifyContent:'space-between'}}>
          <Button type='primary' disabled={this.props.step === 0} onClick={this.stepBackward}>Previous</Button>
          <Button type='primary' disabled={this.props.step === this.props.maxStep} onClick={this.stepForward}>Next</Button>
        </div>
      </div>
    )
  }

}

export default ConfigCreator
