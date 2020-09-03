import React from 'react';
import { Input, Form } from 'antd';
const { TextArea } = Input;


class Basic extends React.Component {

  render() {
    return (
      <>
        <h5 style={{backgroundColor:'#e8e8e8', borderRadius:3, padding:10}}>Specify basic information about the robot. The robot name will be used to specify file names.</h5>
        <Form initialValues={{ urdf:       this.props.urdf,
                               robotName:  this.props.robotName,
                               fixedFrame: this.props.fixedFrame }}>
         <Form.Item
            label="Robot Name"
            name="robotName"
            rules={[{ required: true, message: 'Please provide the robot name' }]}
          >
            <Input defaultValue={this.props.robotName} onChange={this.props.updateRobotName}/>
          </Form.Item>
          <Form.Item
             label="Fixed Frame"
             name="fixedFrame"
             rules={[{ required: true, message: "Please provide the robot's fixed frame" }]}
           >
             <Input defaultValue={this.props.fixedFrame} onChange={this.props.updateFixedFrame}/>
           </Form.Item>
          <Form.Item
             label="URDF"
             name="urdf"
             rules={[{ required: true, message: 'Please provide the urdf contents' }]}
           >
             <TextArea rows={10} onChange={this.props.updateUrdf}/>
           </Form.Item>
        </Form>
      </>
    )
  }

}

export default Basic
