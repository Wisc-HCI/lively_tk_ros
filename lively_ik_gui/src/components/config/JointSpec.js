import React from 'react';
import { List } from 'antd';
import ScalarInput from '../../util/ScalarInput';

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

  render() {
    return (
      <List header={null} footer={null} bordered dataSource={this.props.names.map((item,idx)=>idx)}
            renderItem={(idx)=>(<List.Item key={this.props.names[idx]} label={this.props.names[idx]}>
                                  <List.Item.Meta title={this.props.names[idx]}
                                                  description={<ScalarInput
                                                                  onChange={(v)=>this.debounce(this.updateJointsAtIdx(idx,v))}
                                                                  value={this.props.joints[idx]}
                                                                  step={0.01}
                                                                  showInput={true}
                                                                  min={+this.props.limits[idx][0].toFixed(2)}
                                                                  max={+this.props.limits[idx][1].toFixed(2)}
                                                                />}
                                  />
                                </List.Item>
      )
    }/>)

  }
}

export default JointSpec
