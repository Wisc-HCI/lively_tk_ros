import React from 'react';
import { message, Card, Steps, Divider, Button } from 'antd';
import Basic from './config/Basic';
import Joints from './config/Joints';
import Initial from './config/Initial';
import Collision from './config/Collision';
import Objects from './config/Objects';
import Objectives from './config/Objectives';
import Misc from './config/Misc';
import Preprocessing from './config/Preprocessing';
const { Step } = Steps;

const clearedState = {app:{
                        step:0,
                        canStep:false,
                        displayedState:[]
                      },
                      config:{
                        urdf:null,
                        robotName:null,
                        fixedFrame:null,
                        jointOrdering:[],
                        jointNames:[],
                        eeFixedJoints:[],
                        objectives:[],
                        fixedFrameNoise:0,
                        startingConfig:[],
                        jsDefine:null,
                        axisTypes:[],
                        dispOffsets:[],
                        rotOffsets:[],
                        jointTypes:[],
                        jointLimits:[],
                        velocityLimits:[],
                        robotLinkRadius:null,
                        sampleStates:[],
                        trainingStates:[],
                        problemStates:[],
                        boxes:[],
                        spheres:[],
                        ellipsoids:[],
                        capsules:[],
                        cylinders:[],
                        allFixedJoints:[],
                        allDynamicJoints:[],
                        allLinks:[]
                      }
                     }

class ConfigCreator extends React.Component {

  constructor(props) {
    super(props);
    this.state = clearedState;
  }

  componentDidMount() {
    this.props.socket.on('app_update_response',(data)=>{
      console.log(data.action)
      console.log(data)
      if ((data.action === 'config_update' || data.action === 'fetch') && data.success) {
        // Update config from backend success
        this.setState({config:data.config,app:data.app})
      } else if (data.action === 'config_update' && !data.success) {
        // Update config from backend failed
        message.error(data.message);
      } else if (data.action === 'can_step' || data.action === 'step') {
        this.setState({app:data.app})
      } else {
        console.log(data.message)
        message.error(data.message);
      }
    });
    this.props.socket.emit('app_update',{action:'fetch'})
  }

  reset = () => {
    this.props.socket.emit('app_update',{action:'config_update',...clearedState})
  }

  stepForward = () => {
    this.props.socket.emit('app_update',{action:'step',direction:'forward'})
  }

  stepBackward = () => {
    this.props.socket.emit('app_update',{action:'step',direction:'backward'})
  }

  updateRobotName = (event) => {
    let robotName = event.target.value
    if (robotName === '') {
      robotName = null;
    }
    this.props.socket.emit('app_update',{action:'config_update',config:{robotName:robotName}})
  }

  updateFixedFrame = (event) => {
    let fixedFrame = event.target.value
    if (fixedFrame === '') {
      fixedFrame = null;
    }
    this.props.socket.emit('app_update',{action:'config_update',config:{fixedFrame:fixedFrame}})
  }

  updateUrdf = (event) => {
    let urdf = event.target.value;
    if (urdf === '') {
      urdf = null;
    }
    this.props.socket.emit('app_update',{action:'config_update',config:{urdf:urdf}})
  }

  updateJointOrdering = (jointOrdering) => {
    this.props.socket.emit('app_update',{action:'config_update',config:{jointOrdering:jointOrdering}})
  }

  updateJointNames = (jointNames) => {
    this.props.socket.emit('app_update',{action:'config_update',config:{jointNames:jointNames}})
  }

  updateEeFixedJoints = (eeFixedJoints) => {
    this.props.socket.emit('app_update',{action:'config_update',config:{eeFixedJoints:eeFixedJoints}})
  }

  updateStartingConfig = (startingConfig) => {
    this.props.socket.emit('app_update',{action:'config_update',app:{displayedState:startingConfig},config:{startingConfig:startingConfig}})
  }

  updateDisplayedState = (displayedState) => {
    this.props.socket.emit('app_update',{action:'config_update',app:{displayedState:displayedState},config:{}})
  }

  updateSampleStates = (sampleStates) => {
    this.props.socket.emit('app_update',{action:'config_update',config:{sampleStates:sampleStates}})
  }

  updateTrainingStates = (trainingStates) => {
    this.props.socket.emit('app_update',{action:'config_update',config:{trainingStates:trainingStates}})
  }

  updateProblemStates = (problemStates) => {
    this.props.socket.emit('app_update',{action:'config_update',config:{problemStates:problemStates}})
  }

  updateObjectives = (objectives) => {
    this.props.socket.emit('app_update',{action:'config_update',config:{objectives:objectives}})
  }

  getPage = () => {
    switch (this.state.app.step) {
      case 0:
        return (
          <Basic urdf={this.state.config.urdf}
                 robotName={this.state.config.robotName}
                 fixedFrame={this.state.config.fixedFrame}
                 allLinks={this.state.config.allLinks}
                 updateRobotName={(e)=>this.updateRobotName(e)}
                 updateFixedFrame={(e)=>this.updateFixedFrame(e)}
                 updateUrdf={(e)=>this.updateUrdf(e)}/>
        );
      case 1:
        return (
          <Joints
                  jointOrdering={this.state.config.jointOrdering}
                  jointNames={this.state.config.jointNames}
                  eeFixedJoints={this.state.config.eeFixedJoints}
                  allFixedJoints={this.state.config.allFixedJoints}
                  allDynamicJoints={this.state.config.allDynamicJoints}
                  updateJointOrdering={(list)=>this.updateJointOrdering(list)}
                  updateJointNames={(names)=>this.updateJointNames(names)}
                  updateEeFixedJoints={(names)=>this.updateEeFixedJoints(names)}/>
        );
      case 2:
        return (
          <Initial jointOrdering={this.state.config.jointOrdering}
                   jointLimits={this.state.config.jointLimits}
                   startingConfig={this.state.config.startingConfig}
                   updateStartingConfig={(state)=>this.updateStartingConfig(state)}/>
        );
      case 3:
        return (
          <Collision jointOrdering={this.state.config.jointOrdering}
                     jointLimits={this.state.config.jointLimits}
                     displayedState={this.state.app.displayedState}
                     sampleStates={this.state.config.sampleStates}
                     trainingStates={this.state.config.trainingStates}
                     problemStates={this.state.config.problemStates}
                     updateDisplayedState={(state)=>this.updateDisplayedState(state)}
                     updateSampleStates={(states)=>this.updateSampleStates(states)}
                     updateTrainingStates={(states)=>this.updateTrainingStates(states)}
                     updateProblemStates={(states)=>this.updateProblemStates(states)}/>
        );
      case 4:
        return (
          <Objects/>
        );
      case 5:
        return (
          <Objectives objectives={this.state.config.objectives}
                      eeFixedJoints={this.state.config.eeFixedJoints}
                      jointOrdering={this.state.config.jointOrdering}
                      updateObjectives={(objectives)=>this.updateObjectives(objectives)}/>
        );
      case 6:
        return (
          <Misc/>
        )
      case 7:
        return (
          <Preprocessing/>
        )
      default:
        return;
    }
  }

  render() {
    return (
      <Card title="Config Creator" size='small' style={{margin:10}} extra={<Button onClick={this.reset}>Clear</Button>}>
        <Steps current={this.state.app.step} size="small">
          <Step title="Basic"/>
          <Step title="Joints"/>
          <Step title="Initial"/>
          <Step title="Collision"/>
          <Step title="Objects"/>
          <Step title="Objectives"/>
          <Step title="Misc"/>
          <Step title="Preprocessing"/>
        </Steps>
        <div style={{margin:20}}>
          {this.getPage()}
        </div>
        <Divider/>
        <div style={{display:'flex',justifyContent:'space-between'}}>
          <Button type='primary' disabled={this.state.app.step === 0} onClick={this.stepBackward}>Previous</Button>
          <Button type='primary' disabled={!this.state.app.canStep} onClick={this.stepForward}>Next</Button>
        </div>
      </Card>
    )
  }

}

export default ConfigCreator
