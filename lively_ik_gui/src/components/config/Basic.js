import React from 'react';
import { Input, Descriptions, Select } from 'antd';
const { TextArea } = Input;
const { Option } = Select;

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
             <Select
               placeholder="Please select"
               value={this.props.fixedFrame}
               onChange={(joint)=>this.props.updateFixedFrame(joint)}
               style={{ width:'100%'}}>
               {this.props.allLinks.map((joint)=>{
                 return <Option key={joint}>{joint}</Option>
               })}
             </Select>
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
