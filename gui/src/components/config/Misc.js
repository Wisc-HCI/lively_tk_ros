import React from 'react';
import { Descriptions, InputNumber, Select, Input } from 'antd';
const { Option } = Select;
const { TextArea } = Input;

class Misc extends React.Component {

  render() {
    return (
      <>
        <h5 style={{backgroundColor:'#e8e8e8', borderRadius:3, padding:10}}>
          Miscellaneous configuration parameters.
        </h5>
        <Descriptions column={1}>
          <Descriptions.Item label='Control Mode'>
            <Select style={{width:200}} onChange={(e)=>this.props.updateMode(e)} value={this.props.mode}>
              <Option value='absolute'>Absolute</Option>
              <Option value='relative'>Relative</Option>
            </Select>
          </Descriptions.Item>
          <Descriptions.Item label='Base Noise Scaling'>
            <InputNumber style={{width:200}} onChange={(e)=>this.props.updateFixedFrameNoiseScale(e)} value={this.props.fixedFrameNoiseScale}/>
          </Descriptions.Item>
          <Descriptions.Item label='Base Noise Frequency'>
            <InputNumber style={{width:200}} onChange={(e)=>this.props.updateFixedFrameNoiseFrequency(e)} value={this.props.fixedFrameNoiseFrequency}/>
          </Descriptions.Item>
          <Descriptions.Item label="Joint State Function">
             <TextArea value={this.props.jsDefine} rows={10} onChange={(e)=>this.props.updateJsDefine(e)}/>
           </Descriptions.Item>
        </Descriptions>
      </>
    )
  }

}

export default Misc
