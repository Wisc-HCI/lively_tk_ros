import React from 'react';
import { List, Drawer } from 'antd';
import ScalarInput from '../../util/ScalarInput';

class JointSpec extends React.Component {

  debounce = (func, timeout = 200) => {
    let timer;
    return (...args) => {
      clearTimeout(timer);
      timer = setTimeout(() => { func.apply(this, args); }, timeout);
    };
  }

  onUpdate = (idx,value,sub) => {
    if (this.props.meta.selected && this.props.meta.selected.type === 'starting_config') {
      let state = [...this.props.config.starting_config];
      state[sub][idx] = value;
      this.props.onUpdate({directive:'update',config:{starting_config:state},meta:{displayed_state:state}})
    } else if (this.props.meta.selected && this.props.meta.selected.type === 'collision_state') {
      let states = [...this.props.config.states];
      states[this.props.meta.selected.idx][sub][idx] = value
      this.props.onUpdate({directive:'update',config:{states:states},meta:{displayed_state:states[this.props.meta.selected.idx]}})
    }
  }

  deselect = () => {
    this.props.onUpdate({directive:'update',meta:{selected:null}})
  }

  visible = () => {
    return this.props.meta.selected && (this.props.meta.selected.type === 'starting_config' || this.props.meta.selected.type === 'collision_state')
  }

  getContents = () => {

    let state;
    if (this.props.meta.selected && this.props.meta.selected.type === 'starting_config') {
      state = [...this.props.config.starting_config];
    } else if (this.props.meta.selected && this.props.meta.selected.type === 'collision_state') {
      state = [...this.props.config.states[this.props.meta.selected.idx]];
    } else {
      state = [[0,0,0],[]]
    }

    return  (<>
          <List header={null} footer={null} bordered dataSource={[0,1,2]}
                renderItem={(idx)=>(<List.Item key={['X','Y','Z'][idx]} label={['X','Y','Z'][idx]}>
                                      <List.Item.Meta title={['X','Y','Z'][idx]}
                                                      description={<ScalarInput
                                                                      onChange={(v)=>this.debounce(this.onUpdate(idx,v,0))}
                                                                      value={state[0][idx]}
                                                                      step={0.01}
                                                                      showInput={true}
                                                                      min={+this.props.config.base_link_motion_bounds[idx][0].toFixed(2)}
                                                                      max={+this.props.config.base_link_motion_bounds[idx][1].toFixed(2)}
                                                                    />}
                                                      />
                                    </List.Item>)
                            }/>
          <List header={null} footer={null} bordered dataSource={this.props.config.starting_config[1].map((item,idx)=>idx)}
                renderItem={(idx)=>(<List.Item key={this.props.config.joint_ordering[idx]} label={this.props.config.joint_ordering[idx]}>
                                      <List.Item.Meta title={this.props.config.joint_ordering[idx]}
                                                      description={<ScalarInput
                                                                      onChange={(v)=>this.debounce(this.onUpdate(idx,v,1))}
                                                                      value={state[1][idx]}
                                                                      step={+((this.props.config.joint_limits[idx][1]-this.props.config.joint_limits[idx][0])/40).toFixed(2)}
                                                                      showInput={true}
                                                                      min={+this.props.config.joint_limits[idx][0].toFixed(2)}
                                                                      max={+this.props.config.joint_limits[idx][1].toFixed(2)}
                                                                    />}
                                                      />
                                    </List.Item>)
                            }/>
          </>)
  }

  render() {
    let title = '';
    if (this.props.meta.selected && this.props.meta.selected.type === 'starting_config') {
      title = 'Set Inital State'
    } else if (this.props.meta.selected && this.props.meta.selected.type === 'starting_config') {
      title = 'Set Collision State'
    }

    return (
      <Drawer
        title={title}
        placement="right"
        closable={true}
        maskClosable={true}
        width='50%'
        onClose={this.deselect}
        visible={this.visible()}
        getContainer={false}
        style={{ position: 'absolute' }}
      >
        {this.visible() && this.getContents()}
      </Drawer>
    )
  }
}

export default JointSpec
