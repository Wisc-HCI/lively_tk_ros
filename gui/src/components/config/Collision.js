import React from 'react';
import { Form, Slider, Button, Popover, Badge } from 'antd';
import { Collapse } from 'antd';
const { Panel } = Collapse;

class Collision extends React.Component {

  componentDidMount() {
    if (this.props.jointOrdering.length !== this.props.displayedState.length) {
      let displayedState = this.props.jointLimits.map((limit)=>{
        let minval = +limit[0].toFixed(2);
        let maxval = +limit[1].toFixed(2);
        let avgval = +([minval,maxval].reduce((a,b)=>a+b)/2.0).toFixed(2);
        return avgval;
      });
      this.props.updateStartingConfig(displayedState);
    }
  }

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

  getSampleStatesDescription = () => {
    return (<>
             A set of collision-free <i>sample states</i> so that the robot can learn what is close to a collision state and what is not. <br/>
             Good candidates for <i>sample states</i> are robot configurations that are somewhat close to collisions states, but do not exhibit a collision.
            </>
           )
  }

  getTrainingStatesDescription = () => {
    return (<>
             Any collision-free or colliding states that you think will be useful in the neural network training process. <br/>
             Good candidates for <i>training states</i> are those that are near the interface between collision regions,  <br/>
             such that the robot can learn through example the difference between near-collision and collision states.
            </>
           )
  }

  getProblemStatesDescription = () => {
    return (<>
             Any colliding states that you think will be particularly useful in the neural network training process. <br/>
             Can include non-collision states if desired. Good candidates for <i>problem states</i> are those you want the robot to pay particular attention to.
             </>
            )
  }

  getNumberStyle = (count) => {
    if (count == 0) {
      return {backgroundColor: '#f5222d'}
    } else if (0 < count < 5) {
      return {backgroundColor: '#52c41a'}
    } else {
      return {backgroundColor: '#52c41a'}
    }
  }

  render() {
    return (
      <>
        <h5 style={{backgroundColor:'#e8e8e8', borderRadius:3, padding:10}}>
          Specify joint states for collision avoidance training.
          These include <Popover content={this.getSampleStatesDescription()}><span style={{color:'#1890ff'}}>Sample States</span></Popover>,
          <Popover content={this.getTrainingStatesDescription()}><span style={{color:'#1890ff'}}>Training States</span></Popover>,
          and <Popover content={this.getProblemStatesDescription()}><span style={{color:'#1890ff'}}>Problem States</span></Popover></h5>
        <span style={{display:'flex',justifyContent:'space-around',marginBottom:40}}>
          <Button style={{flex:1,marginRight:5}}>Add to Sample States</Button>
          <Button style={{flex:1,marginRight:5}}>Add to Training States</Button>
          <Button style={{flex:1}}>Add to Problem States</Button>
        </span>
        <Form>
         {this.props.displayedState.map((joint,idx)=>{
           return this.getJointSlider(idx);
         })}
        </Form>
        <Collapse accordion>
          <Panel header="Sample States" key="1" extra={<Badge showZero count={this.props.sampleStates.length} style={this.getNumberStyle(this.props.sampleStates.length)}/>}>
            <p>SHOW SAMPLE STATES HERE</p>
          </Panel>
          <Panel header="Training States" key="2" extra={<Badge showZero count={this.props.trainingStates.length} style={this.getNumberStyle(this.props.trainingStates.length)}/>}>
            <p>SHOW TRAINING STATES HERE</p>
          </Panel>
          <Panel header="Problem States" key="3" extra={<Badge showZero count={this.props.problemStates.length} style={this.getNumberStyle(this.props.problemStates.length)}/>}>
            <p>SHOW PROBLEM STATES HERE</p>
          </Panel>
        </Collapse>
      </>
    )
  }

}

export default Collision
