import React from 'react';
import { Input, Descriptions } from 'antd';
const { TextArea } = Input;


class Basic extends React.Component {

  render() {
    return (
      <>
        <h5 style={{backgroundColor:'#e8e8e8', borderRadius:3, padding:10}}>Specify basic information about the robot. The robot name will be used to specify file names.</h5>
        <Descriptions column={2}>
         <Descriptions.Item label="Robot Name">
            <Input value={this.props.robotName} defaultValue={this.props.robotName} onChange={this.props.updateRobotName}/>
          </Descriptions.Item>
          <Descriptions.Item label="Fixed Frame">
             <Input value={this.props.fixedFrame} defaultValue={this.props.fixedFrame} onChange={this.props.updateFixedFrame}/>
           </Descriptions.Item>
          <Descriptions.Item label="URDF">
             <TextArea value={this.props.urdf} defaultValue={this.props.urdf} rows={10} onChange={this.props.updateUrdf}/>
           </Descriptions.Item>
        </Descriptions>
      </>
    )
  }

}

export default Basic
