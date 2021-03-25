import React from 'react';
import { Input, Select, Modal, Tabs, Button,
         Badge, Alert, Space, Tag, Card, List,
         Tooltip } from 'antd';
import { EditOutlined, DeleteOutlined } from '@ant-design/icons';
import DualSlider from '../../util/DualSlider';
const { TabPane } = Tabs;
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

  tagRender = (props) => {
    const { label, closable, onClose } = props;

    let color = '#BE33FF';
    let tooltip = 'This joint is not used as part of a chain, but can still be controlled';
    this.props.config.joint_names.forEach((chain)=>{
      chain.forEach(joint=>{
        if (joint === label) {
          color = '#1890ff'
          tooltip = 'This joint is part of a chain';
        }
      })
    })

    return (
      <Tooltip title={tooltip} style={{ marginRight: 3 }} color={color}>
        <Tag color={color} closable={closable} onClose={onClose}>
          {label}
        </Tag>
      </Tooltip>
      );
  }

  updateBaseLinkMotionBounds = (value,idx) => {
    let bounds = [...this.props.config.base_link_motion_bounds];
    bounds[idx] = value;
    this.props.updateBaseLinkMotionBounds(bounds)
  }

  selectState = () => {
    this.props.updateMeta({selected:{idx:null,type:'starting_config'},displayed_state:this.props.config.starting_config})
  }

  render() {
    return (
      <>
        <Tabs defaultActiveKey='1' tabPosition='left' style={{ height: '100%', width:'100%' }}>
          <TabPane tab="URDF" key="1">
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
              style={{ width:'100%',paddingBottom:10.}}>
              {this.props.meta.links.map((joint)=>{
                return <Option key={joint}>{joint}</Option>
              })}
            </Select>
            <h3>Base Position</h3>
            <DualSlider showInput={true} min={-5} max={5} step={0.01}
                        value={this.props.config.base_link_motion_bounds[0]}
                        onChange={(v)=>this.updateBaseLinkMotionBounds(v,0)}/>
            <DualSlider showInput={true} min={-5} max={5} step={0.01}
                        value={this.props.config.base_link_motion_bounds[1]}
                        onChange={(v)=>this.updateBaseLinkMotionBounds(v,1)}/>
            <DualSlider showInput={true} min={-5} max={5} step={0.01}
                        value={this.props.config.base_link_motion_bounds[2]}
                        onChange={(v)=>this.updateBaseLinkMotionBounds(v,2)}/>

          </TabPane>
          <TabPane tab="Modes" key="2">
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
          </TabPane>
          <TabPane tab="Active Joints" key="3" style={{height:'100%'}}>
            <Select
                mode="multiple"
                placeholder="Please select"
                tagRender={this.tagRender}
                defaultValue={this.props.config.joint_ordering}
                onChange={(joints)=>this.props.updateJointOrdering(joints)}
                style={{ width:'100%',marginBottom:10}}
                >
                {this.getOptions(this.props.meta.dynamic_joints)}
            </Select>
          </TabPane>
          <TabPane tab="Chains" key="4" style={{height:'100%', overflow:'auto'}}>
            <List bordered header={null} style={{marginBottom:10}} footer={<Button type='primary' onClick={this.addChain}>Add Chain</Button>}
                  dataSource={this.props.config.joint_names.map((item,idx)=>idx)}
                  renderItem={(idx)=>{
                    return (
                      <List.Item extra={
                        <><Select
                          showSearch
                          style={{ marginLeft:10,maxWidth: 250}}
                          defaultValue={this.props.config.ee_fixed_joints[idx]}
                          placeholder="EE Fixed Joint"
                          optionFilterProp="children"
                          onChange={(e)=>this.updateEE(e,idx)}
                          >
                          {this.getOptions(this.props.meta.fixed_joints)}
                        </Select>
                        <Button style={{ marginLeft:10}} shape="circle" icon={<DeleteOutlined/>} danger onClick={()=>this.removeChain(idx)}/></>
                      }>
                        <Select
                          key={idx}
                          mode="multiple"
                          placeholder="Please select"
                          defaultValue={this.props.config.joint_names[idx]}
                          onChange={(e)=>this.changeChain(e,idx)}
                          style={{ width:'100%', flex:1}}
                          >
                          {this.getOptions(this.props.config.joint_ordering)}
                        </Select>
                      </List.Item>)
                  }}/>
          </TabPane>
          <TabPane tab="Initial" key="5" style={{height:'100%'}}>
            <Card title="Initial States" extra={
              <Tooltip title='Edit'>
                <Button shape="circle" style={{marginLeft:5}} icon={<EditOutlined/>} onClick={this.selectState}/>
              </Tooltip>
            }>
                {this.props.config.starting_config[1].map((value)=><Tag>{value.toFixed(2)}</Tag>)}
            </Card>
          </TabPane>
        </Tabs>
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
