import React, { Component } from 'react';
import { message, Button, Upload, Modal, Divider } from 'antd';
import { UploadOutlined } from '@ant-design/icons';
import YAML from 'yaml';
import {defaultConfig} from '../util/Default';
const { Dragger } = Upload;

class Uploader extends Component {

  handleUpload = async (info) => {
    if (info.file) {
      const reader = new FileReader();
      reader.onabort = () => {message.error('Upload Aborted')}
      reader.onerror = () => {message.error('Upload Error')}
      reader.onload = () => {

        let data = YAML.parse(reader.result);
        if (data) {
          let config = defaultConfig;
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
            <Dragger name='file' onChange={(info)=>this.handleUpload(info)} accept='text/yaml, text/json'>
              <p className="ant-upload-drag-icon">
                <UploadOutlined/>
              </p>
              <p className="ant-upload-text">Click or drag file to this area to upload</p>
              <p className="ant-upload-hint">
                Upload a Config File
              </p>
            </Dragger>
            <Divider>Or</Divider>
            <Button block style={{height:'100%',width:'100%'}} type="primary" onClick={()=>this.props.onUpload(defaultConfig)}>Start from Scratch</Button>
      </Modal>
    )
  }
}

export default Uploader
