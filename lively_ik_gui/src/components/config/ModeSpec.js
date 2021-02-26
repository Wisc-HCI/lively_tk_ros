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
                  savedName:props.modeName,
                  showNameError:!this.validateName(props.name)};
  }

  setModeCategory = (category) => {
    this.setState({modeCategory:category})
  }

  validateName = (name) => {
    if (name === this.props.modeName) {
      return true
    } else if (this.props.modeNames.indexOf(name) >= 0) {
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
    let value = event.target.value;
    let showNameError = false;
    if (this.validateName(value)) {
      this.props.onUpdate({name:value})
    } else {
      showNameError = true
    }
    this.setState({savedName:value,showNameError:showNameError})
  }

  updateWeightAtIdx = (idx,value) => {
    let weights = [...this.props.targetWeights];
    weights[idx] = value;
    this.props.onUpdate({targetWeights:weights});
  }

  getWeightSlider = (idx) => {
    if (!this.props.weights || !this.props.names) {
      return <></>
    }

    return (
      <List.Item key={this.props.names[idx]} label={this.props.names[idx]}>
        <List.Item.Meta title={this.props.names[idx]}
                        description={<LogSlider min={0} max={100} step={0.01} showInput={true} value={this.props.targetWeights[idx]}
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
               disabled={this.props.modeName === 'default'}
               value={this.props.modeName === 'default' ? 'Default' : this.state.savedName}
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
              <List header={null} footer={null} bordered dataSource={this.props.variants.map((variant,idx)=>idx).filter((idx)=>BEHAVIOR_ATTRIBUTE_GROUPS[groupIdx].indexOf(this.props.variants[idx])>=0)}
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
