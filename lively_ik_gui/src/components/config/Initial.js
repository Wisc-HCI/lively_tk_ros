import React from 'react';
import { Form, Slider } from 'antd';

class Initial extends React.Component {

  componentDidMount() {
    if (this.props.jointOrdering.length !== this.props.startingConfig.length) {
      let startingConfig = this.props.jointLimits.map((limit)=>{
        let minval = +limit[0].toFixed(2);
        let maxval = +limit[1].toFixed(2);
        let avgval = +([minval,maxval].reduce((a,b)=>a+b)/2.0).toFixed(2);
        return avgval;
      });
      this.props.updateStartingConfig(startingConfig);
    }
  }

  updateStartingConfigAtIdx = (idx,value) => {
    console.log(value);
    let joints = [...this.props.startingConfig];
    joints[idx] = value;
    this.props.updateStartingConfig(joints);
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
        <Slider defaultValue={this.props.startingConfig[idx]} marks={marks} min={minval} max={maxval} step={0.01} tooltipVisible onChange={(v)=>this.updateStartingConfigAtIdx(idx,v)}/>
      </Form.Item>
    )
  }

  render() {
    return (
      <>
        <h5 style={{backgroundColor:'#e8e8e8', borderRadius:3, padding:10}}>Specify initial states for each actuated joint.</h5>
        <Form>
         {this.props.jointOrdering.map((joint,idx)=>{
           return this.getJointSlider(idx);
         })}
        </Form>
      </>
    )
  }

}

export default Initial
