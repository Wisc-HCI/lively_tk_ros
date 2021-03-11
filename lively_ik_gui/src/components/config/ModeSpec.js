import React from 'react';
import { List, Input, Tabs, Alert } from 'antd';
import LogSlider from '../../util/LogSlider';
import { BEHAVIOR_ATTRIBUTE_GROUPS,
         BEHAVIOR_ATTRIBUTE_GROUP_NAMES } from '../../util/Categories';
const { TabPane } = Tabs;

class ModeSpec extends React.Component {

  constructor(props) {
    super(props);
    this.state = {modeCategory:'Base',
                  cachedMode:props.modeInfo,
                  showNameError:!this.validateName(props.modeInfo.name)};
  }

  setModeCategory = (category) => {
    this.setState({modeCategory:category})
  }

  validateName = (name) => {
    if (name === this.props.modeInfo.name) {
      return true
    } else if (this.props.modeInfo.names.indexOf(name) >= 0) {
      return false
    } else if (name !== undefined && name.toLowerCase() === 'default') {
      return false
    } else if (name === '') {
      return false
    }
    return true
  }

  debounce = (func, timeout = 200) => {
    let timer;
    return (...args) => {
      clearTimeout(timer);
      timer = setTimeout(() => { func.apply(this, args); }, timeout);
    };
  }

  updateName = (event) => {
    let name = event.target.value;
    let cachedMode = {...this.state.cachedMode};
    let showNameError = false;
    cachedMode.name = name;
    if (!this.validateName(name)) {
      showNameError = true
    }
    this.props.onUpdate({cachedData:cachedMode});
    this.setState({cachedMode:cachedMode,showNameError:showNameError})
  }

  updateWeightAtIdx = (idx,value) => {
    let weights = [...this.state.cachedMode.weights];
    let cachedMode = {...this.state.cachedMode};
    weights[idx] = value;
    cachedMode.weights = weights;
    this.props.onUpdate({cachedData:cachedMode,targetModeWeights:weights});
    this.setState({cachedMode:cachedMode})
  }

  getWeightSlider = (idx) => {
    return (
      <List.Item key={idx} label={this.props.objectives[idx].tag}>
        <List.Item.Meta title={this.props.objectives[idx].tag}
                        description={<LogSlider min={0} max={100} step={0.01} showInput={true} value={this.state.cachedMode.weights[idx]}
                                                onChange={(v)=>this.debounce(this.updateWeightAtIdx(idx,v))}/>}
        />
      </List.Item>
    )
  }

  render() {
    return (
      <>
        <h3>Name</h3>
        <Input placeholder='Name this Mode'
               disabled={this.state.cachedMode.name === 'default'}
               value={this.state.cachedMode.name === 'default' ? 'Default' : this.state.cachedMode.name}
               onChange={(v)=>this.debounce(this.updateName(v))}/>
        {this.state.showNameError ? (
          <Alert
             message="Name Error"
             description="Please select a unique name for this mode."
             type="error"
             showIcon
           />
         ) : (<></>)
        }
        <Tabs activeKey={this.state.modeCategory}
              centered
              style={{height: '100%', width:'100%' }}
              onChange={(key)=>this.setModeCategory(key)}>
          {BEHAVIOR_ATTRIBUTE_GROUPS.map((group,groupIdx)=>(
            <TabPane tab={BEHAVIOR_ATTRIBUTE_GROUP_NAMES[groupIdx]} key={BEHAVIOR_ATTRIBUTE_GROUP_NAMES[groupIdx]} style={{ height: '100%', width:'100%' }}>
              <List header={null} footer={null} bordered dataSource={this.props.objectives.map((obj,idx)=>idx).filter((idx)=>BEHAVIOR_ATTRIBUTE_GROUPS[groupIdx].indexOf(this.props.objectives[idx].variant)>=0)}
                    renderItem={(idx)=>this.getWeightSlider(idx)}
              />
            </TabPane>
          ))}
        </Tabs>
      </>
    )
  }

}

export default ModeSpec
