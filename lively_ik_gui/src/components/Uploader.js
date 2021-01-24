import React, { Component } from 'react';
import { message, Button, Upload, Modal, Divider } from 'antd';
import { UploadOutlined } from '@ant-design/icons';
import YAML from 'yaml';
const { Dragger } = Upload;

const emptyConfig = {
  axis_types:[],
  base_link_motion_bounds:[[0,0],[0,0],[0,0]],
  collision_scores:[],
  static_environment:{
    cuboids:[],
    spheres:[],
    pcs:[]
  },
  fixed_frame:'',
  goals:[],
  joint_limits:[],
  joint_names:[],
  joint_ordering:[],
  joint_types:[],
  mode_control:'absolute',
  mode_environment:'ECAA',
  nn_jointpoint:[],
  nn_main:[],
  objectives:[],
  states:[],
  robot_link_radius:0.05,
  rot_offsets:[],
  starting_config:[],
  urdf:'<?xml version="1.0" ?><robot name="default" xmlns:xacro="http://www.ros.org/wiki/xacro"><link name="base_link"/><joint name="default_joint" type="fixed"><parent link="base_link" /><child link="default_link" /><origin xyz="0 0 0" rpy="0 0 0" /></joint><link name="default_link"/></robot>',
  velocity_limits:[],
  disp_offsets:[],
  displacements:[]
}

class Uploader extends Component {

  handleUpload = async (info) => {
    if (info.file) {
      console.log(info.file);
      const reader = new FileReader();
      reader.onabort = () => {message.error('Upload Aborted')}
      reader.onerror = () => {message.error('Upload Error')}
      reader.onload = () => {

        let data = YAML.parse(reader.result);
        if (data) {
          let config = emptyConfig;
          for (const [key,value] of Object.entries(data)) {
            config[key] = value
          }
          this.props.onUpload(config);
        }
      }
      reader.readAsText(info.file.originFileObj)

    }
  }

  render() {
    return (
      <Modal title="Let's Begin!"
          footer={null}
          visible={this.props.visible}
          onCancel={this.props.onCancel}
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
            <Divider>Or</Divider>
            <Button block style={{height:'100%',width:'100%'}} type="primary" onClick={()=>this.props.onUpload(emptyConfig)}>Start from Scratch</Button>
      </Modal>
    )
  }
}

export default Uploader
