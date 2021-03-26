import React from 'react';
import { Tabs, Checkbox, InputNumber, List, Space, Tag, Button, Tooltip, Result, Progress, Badge } from 'antd';
import { EditOutlined, CopyOutlined, DeleteOutlined } from '@ant-design/icons';
const { TabPane } = Tabs;

class Collision extends React.Component {

  deleteState = (idx) => {
    let states = [...this.props.config.states];
    states.splice(idx,1)
    this.props.updateStates(states)
  }

  addState = () => {
    let joints = [...this.props.meta.displayed_state];
    let states = [...this.props.config.states];
    states.push(joints);
    this.props.updateStates(states)
  }

  selectState = (idx) => {
    this.props.updateMeta({selected:{idx:idx,type:'collision_state'},displayed_state:this.props.config.states[idx],control:'manual'})
  }

  copyState = (idx) => {
    let states = [...this.props.config.states];
    let copied = [...states[idx]]
    states.splice(idx+1,0,copied);
    this.props.updateStates(states)
    this.props.updateMeta({selected:{idx:idx+1,type:'collision_state'},displayed_state:copied,control:'manual'})
  }

  getColor = (idx) => {
    if (this.props.meta.selected === null) {
      return 'white'
    } else if (idx === this.props.meta.selected.idx) {
      return '#E9E9E9'
    }
  }

  getNNStatus = () => {
    if (this.props.meta.nn_utd) {
      return 'success'
    } else if (this.props.meta.nn_progress > 0) {
      return 'info'
    } else {
      return 'warning'
    }
  }

  getNNIcon = () => {
    if (this.props.meta.nn_utd) {
      return null
    } else if (this.props.meta.nn_progress > 0) {
      return <Progress type="circle" percent={this.props.meta.nn_progress} width={80} />
    } else {
      return null
    }
  }

  getNNTitle = () => {
    if (this.props.meta.nn_utd) {
      return 'Collision Neural Network Up-To-Date!'
    } else if (this.props.meta.nn_progress > 0) {
      return 'Training...'
    } else {
      return 'Re-Training Needed!'
    }
  }

  getNNExtra = () => {
    if (this.props.meta.nn_utd) {
      return <Button type="primary" onClick={()=>this.props.beginTraining()}>Train</Button>
    } else if (this.props.meta.nn_progress > 0) {
      return <Button disabled type="primary" onClick={()=>this.props.beginTraining()}>Train</Button>
    } else {
      return <Button type="primary" onClick={()=>this.props.beginTraining()}>Train</Button>
    }
  }

  render() {
    return (
      <Tabs defaultActiveKey='1' tabPosition='left' style={{ height: '100%', width:'100%' }}>
        <TabPane tab='Link Collision' key="1">
          <h3>Robot Link Radius</h3>
          <Space style={{display:'flex',paddingBottom:10}}>
            <InputNumber min={0} max={10} value={this.props.config.robot_link_radius} step={0.01} onChange={(value)=>this.props.updateRobotLinkRadius(value)} />
            <Checkbox
              checked={this.props.meta.show_link_collision}
              onChange={(e)=>this.props.updateShowLinkCollision(e.target.checked)}
              >Show Link Collision</Checkbox>
          </Space>
        </TabPane>
        <TabPane tab='Environment' key='2'>
          <h3>Spheres</h3>
          <List header={null} footer={<Button type="primary" onClick={()=>{}}>Add Sphere</Button>} bordered dataSource={this.props.config.static_environment.spheres} style={{marginBottom:10}}
                renderItem={(item)=>(
                  <List.Item>{item.name}</List.Item>
                )}
          />
          <h3>Cuboids</h3>
          <List header={null} footer={<Button type="primary" onClick={()=>{}}>Add Cuboid</Button>} bordered dataSource={this.props.config.static_environment.cuboids}
                renderItem={(item)=>(
                  <List.Item>{item.name}</List.Item>
                )}
          />
        </TabPane>
        <TabPane tab='Training States' key='3'>
          <List header={null} footer={<Button type="primary" onClick={this.addState}>Add State</Button>}
                bordered dataSource={this.props.config.states.map((state,idx)=>idx)}
                style={{width:'100%'}}
                renderItem={(state,idx)=>(
                  <List.Item style={{width:'100%',backgroundColor:this.getColor(idx)}} extra={
                      <Space style={{width: 120}}>
                        <Tooltip title='Edit'>
                          <Button shape="circle" style={{marginLeft:5}} icon={<EditOutlined/>} onClick={()=>this.selectState(idx)}/>
                        </Tooltip>
                        <Tooltip title='Copy'>
                          <Button shape="circle" style={{marginLeft:5}} icon={<CopyOutlined/>} onClick={()=>this.copyState(idx)}/>
                        </Tooltip>
                        <Tooltip title='Delete' color='#ff4d4f'>
                          <Button shape="circle" style={{marginLeft:5}} icon={<DeleteOutlined/>} danger onClick={()=>this.deleteState(idx)}/>
                        </Tooltip>
                      </Space>
                    }>
                    <div style={{width:'100%'}}>
                      {this.props.config.states[idx][1].map((value,i)=>(
                        <Tag key={this.props.config.joint_ordering[i]} style={{margin: 1}}>
                          {value.toFixed(2)}
                        </Tag>
                      ))}
                    </div>

                  </List.Item>
                )}
          />
        </TabPane>
        <TabPane tab={(this.props.meta.nn_utd) ? 'Training' : <Badge style={{ backgroundColor: '#faad14' }} offset={[14,0]} count='!'>Training</Badge>} key='4'>
        <Result
          status={this.getNNStatus()}
          icon={this.getNNIcon()}
          title={this.getNNTitle()}
          extra={this.getNNExtra()}
          />
        </TabPane>
      </Tabs>
    )
  }

}

export default Collision
