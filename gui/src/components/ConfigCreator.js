import React from 'react';
import { message, Card, Steps, Divider, Button, Upload, Modal } from 'antd';
import { UploadOutlined } from '@ant-design/icons';
import YAML from 'yaml'
import Basic from './config/Basic';
import Joints from './config/Joints';
import Initial from './config/Initial';
import Collision from './config/Collision';
import Objects from './config/Objects';
import Objectives from './config/Objectives';
import Misc from './config/Misc';
import Preprocessing from './config/Preprocessing';
const { Step } = Steps;
const { Dragger } = Upload;

const toCamel = (s) => {
  return s.replace(/([-_][a-z])/ig, ($1) => {
    return $1.toUpperCase()
      .replace('-', '')
      .replace('_', '');
  });
};

const clearedState = {uploaderVisible:false,
                      app:{
                        step:0,
                        canStep:false,
                        displayedState:[],
                        preprocesssingState:{write_yaml:0.0,
                                             julia_nn:0.0,
                                             julia_params:0.0,
                                             python:0.0}
                      },
                      config:{
                        urdf:null,
                        robotName:null,
                        fixedFrame:'world',
                        jointOrdering:[],
                        jointNames:[],
                        eeFixedJoints:[],
                        objectives:[],
                        fixedFrameNoise:0,
                        startingConfig:[],
                        axisTypes:[],
                        dispOffsets:[],
                        rotOffsets:[],
                        jointTypes:[],
                        jointLimits:[],
                        velocityLimits:[],
                        robotLinkRadius:0.05,
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
    this.props.socket.on('app_process_response',(data)=>{
      if (data.success) {
        console.log(data)
        let state = {};
        if (data.config) {
          state.config = data.config;
        }
        if (data.app) {
          state.app = data.app;
        }
        this.setState(state)
      }
    })
    this.props.socket.emit('app_update',{action:'fetch'})
  }

  handleModalCancel = () => {
    this.setState({uploaderVisible:false})
  }

  openUploader = ()  => {
    this.setState({uploaderVisible:true})
  }

  handleUpload = async (info) => {
    if (info.file) {
      console.log(info.file);
      const reader = new FileReader();
      reader.onabort = () => {message.error('Upload Aborted')}
      reader.onerror = () => {message.error('Upload Error')}
      reader.onload = () => {

        let data = YAML.parse(reader.result);
        if (data) {
          let state = clearedState;
          for (const [key,value] of Object.entries(data)) {
            state.config[toCamel(key)] = value
          }
          state.app.displayedState = data.starting_config;
          this.props.socket.emit('app_update',{action:'config_update',...state});
          this.setState({uploaderVisible:false});
        }
      }
      reader.readAsText(info.file.originFileObj)

    }
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

  updateFixedFrame = (fixedFrame) => {
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

  updateBoxes = (boxes) => {
    this.props.socket.emit('app_update',{action:'config_update',config:{boxes:boxes}})
  }

  updateSpheres = (spheres) => {
    this.props.socket.emit('app_update',{action:'config_update',config:{spheres:spheres}})
  }

  updateEllipsoids = (ellipsoids) => {
    this.props.socket.emit('app_update',{action:'config_update',config:{ellipsoids:ellipsoids}})
  }

  updateCapsules = (capsules) => {
    this.props.socket.emit('app_update',{action:'config_update',config:{capsules:capsules}})
  }

  updateCylinders = (cylinders) => {
    this.props.socket.emit('app_update',{action:'config_update',config:{cylinders:cylinders}})
  }

  updateMode = (mode) => {
    this.props.socket.emit('app_update',{action:'config_update',config:{mode:mode}})
  }

  updateRobotLinkRadius = (radius) => {
    this.props.socket.emit('app_update',{action:'config_update',config:{robotLinkRadius:radius}})
  }

  updateFixedFrameNoiseScale = (scale) => {
    this.props.socket.emit('app_update',{action:'config_update',config:{fixedFrameNoiseScale:scale}})
  }

  updateFixedFrameNoiseFrequency = (frequency) => {
    this.props.socket.emit('app_update',{action:'config_update',config:{fixedFrameNoiseFrequency:frequency}})
  }

  beginPreprocess = () => {
    this.props.socket.emit('app_process',{action:'preprocess'})
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
          <Objects jointOrdering={this.state.config.jointOrdering}
                   boxes={this.state.config.boxes}
                   spheres={this.state.config.spheres}
                   ellipsoids={this.state.config.ellipsoids}
                   capsules={this.state.config.capsules}
                   cylinders={this.state.config.cylinders}
                   updateBoxes={(objects)=>this.updateBoxes(objects)}
                   updateSpheres={(objects)=>this.updateSpheres(objects)}
                   updateEllipsoids={(objects)=>this.updateEllipsoids(objects)}
                   updateCapsules={(objects)=>this.updateCapsules(objects)}
                   updateCylinders={(objects)=>this.updateCylinders(objects)}/>
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
          <Misc mode={this.state.config.mode}
                robotLinkRadius={this.state.config.robotLinkRadius}
                fixedFrameNoiseScale={this.state.config.fixedFrameNoiseScale}
                fixedFrameNoiseFrequency={this.state.config.fixedFrameNoiseFrequency}
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
    <>
      <Card title="Config Creator" size='small' style={{margin:10}} extra={<><Button style={{marginRight:5}} onClick={this.openUploader}>Upload</Button>
                                                                             <Button onClick={this.reset}>Clear</Button>
                                                                           </>}>
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
      <Modal title="Upload"
          footer={null}
          visible={this.state.uploaderVisible}
          onCancel={this.handleModalCancel}
        >
          <Dragger name='file' onChange={(info)=>this.handleUpload(info)} accept='.yaml'>
            <p className="ant-upload-drag-icon">
              <UploadOutlined/>
            </p>
            <p className="ant-upload-text">Click or drag file to this area to upload</p>
            <p className="ant-upload-hint">
              Upload a Config Yaml File
            </p>
          </Dragger>
        </Modal>
      </>
    )
  }

}

export default ConfigCreator
