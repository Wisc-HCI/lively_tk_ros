import React from 'react';
import { Input, Descriptions, Select, Modal, Collapse, Button, Badge, Alert, Space } from 'antd';
import { DeleteOutlined } from '@ant-design/icons';
const { Panel } = Collapse;
const { TextArea } = Input;
const { Option } = Select;

class Basics extends React.Component {

  state = {urdfWindowVisible:false,};

  openUrdfWindow = () => {
    this.setState({urdfWindowVisible:true})
  }

  closeUrdfWindow = () => {
    this.setState({urdfWindowVisible:false})
  }

  updateEE = (joint, idx) => {
    let newJoints = [...this.props.config.ee_fixed_joints];
    newJoints[idx] = joint;
    this.props.updateEeFixedJoints(newJoints);
  }

  changeChain = (chain, idx) => {
    let jointNames = this.props.config.joint_names;
    jointNames[idx] = chain
    this.props.updateJointNames(jointNames)
  }

  addChain = () => {
    this.props.updateJointNames([...this.props.config.joint_names,[]]);
  }

  removeChain = (idx) => {
    let jointNames = [...this.props.config.joint_names];
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
        <Collapse defaultActiveKey={['1']} accordion={true}>
          <Panel header="URDF" key="1">
            <h3>URDF</h3>
            <Space style={{display:'flex',paddingBottom:10}}>
              <Button onClick={this.openUrdfWindow} size='large'>Update</Button>
              {this.props.meta.valid_urdf ? (
                <Alert message="Valid" type="success" showIcon />
              ) : (
                <Alert message="Invalid" type="error" showIcon />
              )}
            </Space>
            <h3>Fixed Frame</h3>
            <Select
              placeholder="Please select a fixed frame"
              value={this.props.config.fixed_frame}
              onChange={(joint)=>this.props.updateFixedFrame(joint)}
              style={{ width:'100%'}}>
              {this.props.meta.links.map((joint)=>{
                return <Option key={joint}>{joint}</Option>
              })}
            </Select>
          </Panel>
          <Panel header="Modes" key="3">
            <Space style={{display:'flex'}}>
              <div>
                <h3>Control</h3>
                <Select
                  placeholder="Please select a control mode"
                  value={this.props.config.mode_control}
                  onChange={(mode)=>this.props.updateControlMode(mode)}
                  style={{ width:'100%'}}>
                    <Option key='absolute'>Absolute</Option>
                    <Option key='relative'>Relative</Option>
                </Select>
              </div>
              <div>
                <h3>Environment</h3>
                <Select
                  placeholder="Please select an environment mode"
                  value={this.props.config.mode_environment}
                  onChange={(mode)=>this.props.updateEnvironmentMode(mode)}
                  style={{ width:'100%'}}>
                    <Option key='ECA'>ECA</Option>
                    <Option key='ECA3'>ECA3</Option>
                    <Option key='ECAA'>ECAA</Option>
                    <Option key='None'>None</Option>
                </Select>
              </div>
            </Space>
          </Panel>
          <Panel header="Arms and Joints" key="2">
            <h3>Joints</h3>
            <Select
              mode="multiple"
              placeholder="Please select"
              defaultValue={this.props.config.joint_ordering}
              onChange={(joints)=>this.props.updateJointOrdering(joints)}
              style={{ width:'100%'}}
              >
              {this.getOptions(this.props.meta.dynamic_joints)}
            </Select>
            <h3>Chains</h3>
            {this.props.config.joint_names.map((chain, idx)=>{
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
                  {this.getOptions(this.props.config.joint_ordering)}
                </Select>
                <Select
                  showSearch
                  style={{ width: "100%", flex:1, marginLeft:6}}
                  defaultValue={this.props.config.ee_fixed_joints[idx]}
                  placeholder="EE Fixed Joint"
                  optionFilterProp="children"
                  onChange={(e)=>this.updateEE(e,idx)}
                >
                  {this.getOptions(this.props.meta.fixed_joints)}
                </Select>
                <Button danger style={{marginLeft:6}} onClick={()=>this.removeChain(idx)} icon={<DeleteOutlined/>}></Button>
              </span>
            )})}
            <Button type='primary' onClick={this.addChain}>Add Chain</Button>

          </Panel>
        </Collapse>
        <Modal title="Specify URDF"
            footer={null}
            closable={this.props.meta.valid_urdf}
            visible={this.state.urdfWindowVisible}
            onCancel={this.closeUrdfWindow}
            maskClosable={false}
          >
          {this.props.meta.valid_urdf ? (
              <Badge.Ribbon style={{backgroundColor: '#52c41a'}} text='Valid'>
                <TextArea value={this.props.config.urdf} defaultValue={this.props.config.urdf} rows={10} onChange={this.props.updateUrdf}/>
              </Badge.Ribbon>
            ) : (
              <Badge.Ribbon color='red' text='Invalid'>
                <TextArea value={this.props.config.urdf} defaultValue={this.props.config.urdf} rows={10} onChange={this.props.updateUrdf}/>
              </Badge.Ribbon>
            )
          }

        </Modal>
      </>
    )
  }

}

export default Basics
