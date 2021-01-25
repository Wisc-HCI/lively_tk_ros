import React from 'react';
import { Form, Slider, Button, Popover, Badge, Collapse, Checkbox, InputNumber, List, Space, Tag } from 'antd';
const { Panel } = Collapse;

class Collision extends React.Component {

  updateDisplayedStateAtIdx = (idx,value) => {
    console.log(value);
    let joints = [...this.props.displayedState];
    joints[idx] = value;
    this.props.updateDisplayedState(joints);
  }

  getJointSlider = (idx) => {
    let minval = +this.props.jointLimits[idx][0].toFixed(2);
    let maxval = +this.props.jointLimits[idx][1].toFixed(2);
    // let avgval = +([minval,maxval].reduce((a,b)=>a+b)/2.0).toFixed(2);
    let marks = {};
    marks[minval] = minval.toString();
    marks[maxval] = maxval.toString();

    return (
      <Form.Item key={this.props.jointOrdering[idx]} label={this.props.jointOrdering[idx]}>
        <Slider defaultValue={this.props.displayedState[idx]} marks={marks} min={minval} max={maxval} step={0.01} tooltipVisible onChange={(v)=>this.updateDisplayedStateAtIdx(idx,v)}/>
      </Form.Item>
    )
  }

  addToSampleStates = () => {
    let joints = [...this.props.displayedState];
    let sampleStates = [...this.props.config.states];
    sampleStates.push(joints);
    this.props.updateStates(sampleStates)
  }

  render() {
    return (
      <Collapse defaultActiveKey={['1']} accordion={true}>
        <Panel header="Link Collision" key="1">
          <h3>Robot Link Radius</h3>
          <Space style={{display:'flex',paddingBottom:10}}>
            <InputNumber min={0} max={10} defaultValue={0.05} step={0.01} onChange={(value)=>this.props.updateRobotLinkRadius(value)} />
            <Checkbox>Show Link Collision</Checkbox>
          </Space>
        </Panel>
        <Panel header="Environment" key="2">
          <h3>Spheres</h3>
          <List header={null} footer={null} bordered dataSource={this.props.config.static_environment.spheres} style={{marginBottom:10}}
                renderItem={(item)=>(
                  <List.Item>{item.name}</List.Item>
                )}
          />
          <h3>Cuboids</h3>
          <List header={null} footer={null} bordered dataSource={this.props.config.static_environment.cuboids}
                renderItem={(item)=>(
                  <List.Item>{item.name}</List.Item>
                )}
          />
        </Panel>
        <Panel header="Training States" key="3">
          <List header={null} footer={null} bordered dataSource={this.props.config.states}
                renderItem={(item)=>(
                  <List.Item onClick={()=>this.props.updateDisplayedState(item)}>
                    <Space>
                      {item.map((value)=><Tag>{value.toFixed(2)}</Tag>)}
                    </Space>
                  </List.Item>
                )}
          />
        </Panel>
      </Collapse>
    )
  }

}

export default Collision
