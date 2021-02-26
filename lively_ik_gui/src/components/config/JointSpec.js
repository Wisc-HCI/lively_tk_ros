import React from 'react';
import { List, Slider } from 'antd';

class JointSpec extends React.Component {

  debounce = (func, timeout = 200) => {
    let timer;
    return (...args) => {
      clearTimeout(timer);
      timer = setTimeout(() => { func.apply(this, args); }, timeout);
    };
  }

  updateJointsAtIdx = (idx,value) => {
    let joints = [...this.props.joints];
    joints[idx] = value;
    this.props.onUpdate(joints);
  }

  getJointSlider = (idx) => {
    if (!this.props.joints || !this.props.names || !this.props.limits) {
      return <></>
    }

    let minval = +this.props.limits[idx][0].toFixed(2);
    let maxval = +this.props.limits[idx][1].toFixed(2);
    // let avgval = +([minval,maxval].reduce((a,b)=>a+b)/2.0).toFixed(2);
    let marks = {};
    marks[minval] = minval.toString();
    marks[maxval] = maxval.toString();

    return (
      <List.Item key={this.props.names[idx]} label={this.props.names[idx]}>
        <List.Item.Meta title={this.props.names[idx]}
                        description={<Slider defaultValue={this.props.joints[idx]}
                                             marks={marks} min={minval} max={maxval} step={0.01}
                                             tooltipVisible
                                             onChange={(v)=>this.debounce(this.updateJointsAtIdx(idx,v))}/>}
        />
      </List.Item>
    )
  }

  render() {
    return (
      <List header={null} footer={null} bordered dataSource={this.props.names.map((item,idx)=>idx)}
            renderItem={(idx)=>this.getJointSlider(idx)}
      />
    )
  }

}

export default JointSpec
