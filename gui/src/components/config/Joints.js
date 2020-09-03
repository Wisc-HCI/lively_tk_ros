import React from 'react';
import { Input, Form, Tag, Tooltip, Select, Button } from 'antd';
import { PlusOutlined, DeleteOutlined } from '@ant-design/icons';
const { Option } = Select;

class Joints extends React.Component {

  constructor(props) {
    super(props);
    this.state = {inputVisible: false,
                  inputValue: '',
                  editInputIndex: -1,
                  editInputValue: ''};

  }

  updateEE = (joint, idx) => {
    let newJoints = [...this.props.eeFixedJoints];
    newJoints[idx] = joint;
    this.props.updateEeFixedJoints(newJoints);
  }

  changeChain = (chain, idx) => {
    let jointNames = this.props.jointNames;
    jointNames[idx] = chain
    this.props.updateJointNames(jointNames)
  }

  addChain = () => {
    this.props.updateJointNames([...this.props.jointNames,[]]);
  }

  removeChain = (idx) => {
    let jointNames = [...this.props.jointNames];
    jointNames.splice(idx,1);
    this.props.updateJointNames(jointNames)
  }

  getOptions = (joints) => {
    return joints.map((joint) => {
      return (<Option key={joint}>{joint}</Option>)
    })
  }


  render() {
    return (
      <>
        <h5 style={{backgroundColor:'#e8e8e8', borderRadius:3, padding:10}}>Specify joint information about the robot. Joint information provided here will be used to create JointState messages.</h5>
        <Form>
         <Form.Item label="Joints">
           <Select
             mode="multiple"
             placeholder="Please select"
             defaultValue={this.props.jointOrdering}
             onChange={(joints)=>this.props.updateJointOrdering(joints)}
             style={{ width:'100%'}}
             >
             {this.getOptions(this.props.allDynamicJoints)}
           </Select>
         </Form.Item>
         <Form.Item label="Joint Chains">
            {this.props.jointNames.map((chain, idx)=>{
              return (
              <span style={{display:'flex',marginBottom:10}}>
                <Select
                  key={idx}
                  mode="multiple"
                  placeholder="Please select"
                  defaultValue={chain}
                  onChange={(e)=>this.changeChain(e,idx)}
                  style={{ width:'100%', flex:3}}
                  >
                  {this.getOptions(this.props.jointOrdering)}
                </Select>
                <Select
                  showSearch
                  style={{ width: "100%", flex:1, marginLeft:6}}
                  defaultValue={this.props.eeFixedJoints[idx]}
                  placeholder="EE Fixed Joint"
                  optionFilterProp="children"
                  onChange={(e)=>this.updateEE(e,idx)}
                >
                  {this.getOptions(this.props.allFixedJoints)}
                </Select>
                <Button danger style={{marginLeft:6}} onClick={()=>this.removeChain(idx)} icon={<DeleteOutlined/>}></Button>
              </span>
            )})}
            <Button type='primary' onClick={this.addChain}>Add Chain</Button>
          </Form.Item>
        </Form>
      </>
    )
  }

}

export default Joints
