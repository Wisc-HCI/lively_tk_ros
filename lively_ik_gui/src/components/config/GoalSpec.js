import React from 'react';
import { List, Input, Alert } from 'antd';
import TranslationInput from '../../util/TranslationInput';
import RotationInput from '../../util/RotationInput';
import ScalarInput from '../../util/ScalarInput';

class GoalSpec extends React.Component {

  constructor(props) {
    super(props);
    this.state = {cachedGoal:props.goalInfo,
                  showNameError:!this.validateName(props.goalInfo.name)};
  }

  validateName = (name) => {
    if (name === this.props.goalInfo.name) {
      return true
    } else if (this.props.goalNames.indexOf(name) >= 0) {
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

  save = () => {
    this.props.onUpdate({goal:this.state.cachedGoal})
  }

  updateName = (event) => {
    let name = event.target.value;
    let cachedGoal = {...this.state.cachedGoal};
    let showNameError = false;
    cachedGoal.name = name;
    if (!this.validateName(name)) {
      showNameError = true
    }
    this.props.onUpdate({cachedData:cachedGoal});
    this.setState({cachedGoal:cachedGoal,showNameError:showNameError})
  }

  updateValues = (value,valueIdx,featureType,featureIdx) => {
    let values = [...this.state.cachedGoal.values];
    let cachedGoal = {...this.state.cachedGoal};
    values[valueIdx][featureType] = value
    cachedGoal.values = values;
    this.props.onUpdate({cachedData:cachedGoal,targetGoalValues:values});
    this.setState({cachedGoal:cachedGoal})
  }

  getGoalValueSpecifier = (idx) => {
    if (this.state.cachedGoal.values[idx].vector !== undefined) {
      return (
        <List.Item key={idx} label={this.props.objectives[idx].tag}>
          <List.Item.Meta title={this.props.objectives[idx].tag}
                          description={<TranslationInput
                                          step={0.01}
                                          value={this.state.cachedGoal.values[idx].vector}
                                          onChange={(v)=>{this.updateValues(v,idx,'vector')}}
                                          min={[-5,-5,-5]} max={[5,5,5]} showInput={true}/>}/>
        </List.Item>
      )
    } else if (this.state.cachedGoal.values[idx].quaternion !== undefined) {
      return (
        <List.Item key={idx} label={this.props.objectives[idx].tag}>
          <List.Item.Meta title={this.props.objectives[idx].tag}
                          description={<RotationInput
                                          step={0.1}
                                          value={this.state.cachedGoal.values[idx].quaternion}
                                          onChange={(v)=>{this.updateValues(v,idx,'quaternion')}}
                                          showInput={true}/>}/>
        </List.Item>)
    } else if (this.state.cachedGoal.values[idx].scalar !== undefined) {
       return (
        <List.Item key={idx} label={this.props.objectives[idx].tag}>
          <List.Item.Meta title={this.props.objectives[idx].tag}
                          description={<ScalarInput
                                        step={0.01}
                                        min={this.props.jointLimits[this.props.objectives[idx].indices[0]][0]}
                                        max={this.props.jointLimits[this.props.objectives[idx].indices[0]][1]}
                                        value={this.state.cachedGoal.values[idx].scalar}
                                        onChange={(v)=>this.updateValues(v,idx,'scalar')}
                                        showInput={true}/>}/>

        </List.Item>)
    }
  }

  render() {
    return (
      <>
        <h3>Name</h3>
        <Input placeholder='Name this Goal'
               disabled={this.state.cachedGoal.name === 'default'}
               value={this.state.cachedGoal.name === 'default' ? 'Default' : this.state.cachedGoal.name}
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
        <h3 style={{marginTop:10}}>Direction Values</h3>
        <List header={null} footer={null} style={{}} bordered dataSource={this.state.cachedGoal.values.map((value,idx)=>idx)}
              renderItem={(idx)=>this.getGoalValueSpecifier(idx)}
        />
      </>
    )
  }

}

export default GoalSpec
