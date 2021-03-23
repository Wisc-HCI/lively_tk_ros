import React from 'react';
import { List, Slider, Select, Input, InputNumber, Row, Col, Cascader } from 'antd';
import { defaultObjectives, defaultWeights, defaultObjectiveNames } from '../../util/Default';
import { getObjectivePreview } from '../../util/Englishify';
import LogSlider from '../../util/LogSlider';
import { BASE_OBJECTIVES,
         DIRECTION_OBJECTIVES,
         LIVELINESS_OBJECTIVES,
         PAIRED_OBJECTIVES,
         CARTESIAN_OBJECTIVES,
         JOINT_OBJECTIVES } from '../../util/Categories';
const { Option } = Select;

class ObjectiveSpec extends React.Component {

  constructor(props) {
    super(props);
    this.state = {cachedObjective:props.objective,
                  cachedModeWeights:props.modeWeights,
                  cachedGoalValues:props.goalValues};
  }

  debounce = (func, timeout = 200) => {
    let timer;
    return (...args) => {
      clearTimeout(timer);
      timer = setTimeout(() => { func.apply(this, args); }, timeout);
    };
  }

  getVariantOptions = () => {
    let optionNames = [];
    if (BASE_OBJECTIVES.indexOf(this.state.cachedObjective.variant) >= 0) {
      optionNames = BASE_OBJECTIVES;
    } else if (DIRECTION_OBJECTIVES.indexOf(this.state.cachedObjective.variant) >= 0) {
      optionNames = DIRECTION_OBJECTIVES;
    } else {
      optionNames = LIVELINESS_OBJECTIVES;
    }
    return optionNames.map(name=>(
      <Option value={name}>{defaultObjectiveNames[name]}</Option>
    ))
  }

  getIdxOptions = () => {
    let optionNames = [];
    if (JOINT_OBJECTIVES.indexOf(this.state.cachedObjective.variant) >= 0) {
      optionNames = this.props.jointOrdering;
    } else if (CARTESIAN_OBJECTIVES.indexOf(this.state.cachedObjective.variant) >= 0) {
      optionNames = this.props.eeFixedJoints;
    }
    return optionNames.map((name,idx)=>(
      <Option value={idx}>{name}</Option>
    ))
  }

  getIdxValue = (idx) => {
    if (JOINT_OBJECTIVES.indexOf(this.state.cachedObjective.variant) >= 0) {
      return this.props.jointOrdering[idx];
    } else if (CARTESIAN_OBJECTIVES.indexOf(this.state.cachedObjective.variant) >= 0) {
      return this.props.eeFixedJoints[idx];
    }
  }

  updateTag = (e) => {
    let objective = {...this.state.cachedObjective};
    objective.tag = e.target.value;
    this.setState({cachedObjective:objective},()=>{this.props.onUpdate(this.state)})
  }

  getGoalFromCurrent = (variant,idx1,idx2) => {
    switch (variant) {
      case 'position_match':
        let position = this.props.jointPoses[idx1][idx2]['position']
        return {vector:[position.x,position.y,position.z]}
      case 'orientation_match':
        let quaternion = this.props.jointPoses[idx1][idx2]['quaternion']
        return {quaternion:[quaternion.w,quaternion.x,quaternion.y,quaternion.z]}
      case 'joint_match':
        return {scalar:this.props.startingConfig[idx1]}
      default:
        return {}
    }
  }

  updateVariant = (variant) => {
    let currObj = {...this.state.cachedObjective};
    let weights = [...this.state.cachedModeWeights];
    let goalValues = [...this.state.cachedGoalValues];
    let newObj = JSON.parse(JSON.stringify(defaultObjectives[variant]))

    // First handle indices
    const currentIsJointObjective = (JOINT_OBJECTIVES.indexOf(currObj.variant) >= 0);
    const newIsJointObjective = (JOINT_OBJECTIVES.indexOf(newObj.variant) >= 0);
    const currentIsCartesianObjective = (CARTESIAN_OBJECTIVES.indexOf(currObj.variant) >= 0);
    const newIsCartesianObjective = (CARTESIAN_OBJECTIVES.indexOf(newObj.variant) >= 0);
    const currentIsPairedObjective = (PAIRED_OBJECTIVES.indexOf(currObj.variant) >= 0);
    const newIsPairedObjective = (PAIRED_OBJECTIVES.indexOf(newObj.variant) >= 0);

    if ((currentIsCartesianObjective === newIsCartesianObjective) && (currentIsJointObjective === newIsJointObjective) && (currentIsPairedObjective === newIsPairedObjective)) {
      // In cases where the profiles match up just copy them over.
      // Otherwise, keep the defaults of the new one.
      newObj.indices = currObj.indices;
    } else if (newIsCartesianObjective && currentIsPairedObjective && this.props.jointNames.length > 1) {
      newObj.indices = [0, this.props.jointNames[0].length, 1, this.props.jointNames[1].length]
    } else if (newIsCartesianObjective && currentIsPairedObjective) {
      newObj.indices = [0, this.props.jointNames[0].length, 0, this.props.jointNames[0].length-1]
    } else if (newIsCartesianObjective) {
      newObj.indices = [0, this.props.jointNames[0].length]
    } else if (newIsJointObjective && newIsPairedObjective) {
      newObj.indices = [this.props.jointOrdering.length-1,this.props.jointOrdering.length-2]
    } else if (newIsJointObjective) {
      newObj.indices = [this.props.jointOrdering.length-1]
    }

    // Copy over other fields
    Object.keys(newObj).forEach(field=>{
      if (field !== 'indices' && field !== 'variant' && currObj[field] !== undefined) {
        newObj[field] = currObj[field]
      }
    })

    // Update the weight across all the modes
    weights.forEach((weight,idx)=>{
      weights[idx] = defaultWeights[variant]
    })

    // Update goal values across all the goals
    goalValues.forEach((value,idx)=>{
      goalValues[idx] = this.getGoalFromCurrent(variant,newObj.indices[0],newObj.indices[1])
    })

    // console.log(newObj);
    let cache = {cachedObjective:newObj,cachedModeWeights:weights,cachedGoalValues:goalValues};
    this.setState(cache,()=>{this.props.onUpdate({cachedData:this.state})})
  }

  updateIndices = (values) => {
    let objective = {...this.state.cachedObjective};
    let goalValues = [...this.props.cachedGoalValues];
    objective.indices = values;

    // Update goal values across all the goals
    goalValues.forEach((value,idx)=>{
      goalValues[idx] = this.getGoalFromCurrent(objective.variant,objective.indices[0],objective.indices[1])
    })

    // this.props.onUpdate({objective:objective,goalValues:goalValues})
    let cache = {cachedObjective:objective,cachedGoalValues:goalValues};
    this.setState(cache,()=>{this.props.onUpdate({cachedData:this.state})})
  }

  updateScale = (value) => {
    if (!Number.isNaN(value)) {
      if (typeof(value) == 'string') {
        value = parseFloat(value)
      }
      let objective = {...this.state.cachedObjective};
      objective.scale = (value);
      let cache = {cachedObjective:objective};
      this.setState(cache,()=>{this.props.onUpdate({cachedData:this.state})})
    } else {
      console.log(`Ignoring erroneous input ${value}`)
    }
  }

  updateShape = (value,idx) => {
    if (!Number.isNaN(value)) {
      if (typeof(value) == 'string') {
        value = parseFloat(value)
      }
      let objective = {...this.state.cachedObjective};
      objective.shape[idx] = value;
      let cache = {cachedObjective:objective};
      this.setState(cache,()=>{this.props.onUpdate({cachedData:this.state})})
    } else {
      console.log(`Ignoring erroneous input ${value}`)
    }
  }

  updateFrequency = (value) => {
    if (!Number.isNaN(value)) {
      if (typeof(value) == 'string') {
        value = parseFloat(value)
      }
      let objective = {...this.state.cachedObjective};
      objective.frequency = value;
      let cache = {cachedObjective:objective,cachedModeWeights:this.state.cachedWeights,cachedGoalValues:this.state.cachedGoalValues}
      this.setState(cache,()=>{this.props.onUpdate({cachedData:this.state})})
    } else {
      console.log(`Ignoring erroneous input ${value}`)
    }
  }

  getIdxOptionsFromJointNames = () => {
    let options = [];
    this.props.jointNames.forEach((chain,chain_idx) => {
      let children = [];
      chain.forEach((joint,joint_idx)=>{
        children.push({value:joint_idx,label:joint});
      })
      children.push({value:chain.length,label:this.props.eeFixedJoints[chain_idx]})
      options.push({value:chain_idx,label:`Chain ${chain_idx+1}`,children:children})
    })
    return options
  }

  getIdxPlaceholder = () => {
    if (JOINT_OBJECTIVES.indexOf(this.state.cachedObjective.variant) >= 0) {
      return 'Select the joint';
    } else if (CARTESIAN_OBJECTIVES.indexOf(this.state.cachedObjective.variant) >= 0) {
      return 'Select the arm\'s end effector';
    }
  }

  updateWeightAtIdx = (idx,value) => {
    if (!Number.isNaN(value)) {
      if (typeof(value) == 'string') {
        value = parseFloat(value)
      }
      let modeWeights = [...this.state.cachedModeWeights];
      modeWeights[idx] = value
      let cache = {cachedModeWeights:modeWeights}
      this.setState(cache,()=>{this.props.onUpdate({cachedData:this.state})})
    } else {
      console.log(`Ignoring erroneous input ${value}`)
    }
  }

  getWeightSlider = (idx) => {
    return (
      <List.Item key={this.props.modeNames[idx]}>
        <List.Item.Meta title={this.props.modeNames[idx] === 'default' ? 'Default' : this.props.modeNames[idx]}
                        description={<LogSlider min={0} max={100} step={0.01} showInput={true} value={this.state.cachedModeWeights[idx]}
                                                onChange={(v)=>this.debounce(this.updateWeightAtIdx(idx,v))}/>}
        />
      </List.Item>
    )
  }

  getIndexSettings = () => {
    let settings = [];
    if ((CARTESIAN_OBJECTIVES).indexOf(this.state.cachedObjective.variant) >= 0) {
      if (PAIRED_OBJECTIVES.indexOf(this.state.cachedObjective.variant) >= 0) {
        settings.push(
          <>
            <h3 style={{marginTop:10}}>Cartesian-Controlled Joint</h3>
            <Cascader style={{ width:'100%'}}
                      expandTrigger="hover"
                      value={[this.state.cachedObjective.indices[0],this.state.cachedObjective.indices[1]]}
                      options={this.getIdxOptionsFromJointNames()}
                      onChange={(v)=>{
                        let current = [...this.state.cachedObjective.indices];
                        current[0] = v[0]
                        current[1] = v[1]
                        this.updateIndices(current)
                      }}
            />
            <h3 style={{marginTop:10}}>Reference Joint</h3>
            <Cascader style={{ width:'100%'}}
                      expandTrigger="hover"
                      value={[this.state.cachedObjective.indices[2],this.state.cachedObjective.indices[3]]}
                      options={this.getIdxOptionsFromJointNames()}
                      onChange={(v)=>{
                        let current = [...this.state.cachedObjective.indices];
                        current[2] = v[0]
                        current[3] = v[1]
                        this.updateIndices(current)
                      }}
            />
          </>
        )
      } else {
        settings.push(
          <>
            <h3 style={{marginTop:10}}>Cartesian-Controlled Joint</h3>
            <Cascader style={{ width:'100%'}}
                      expandTrigger="hover"
                      value={this.state.cachedObjective.indices}
                      options={this.getIdxOptionsFromJointNames()}
                      onChange={v=>this.updateIndices(v)}
            />
          </>
        )
      }
    } else if ((JOINT_OBJECTIVES).indexOf(this.state.cachedObjective.variant) >= 0) {
      if (PAIRED_OBJECTIVES.indexOf(this.state.cachedObjective.variant) >= 0) {
        settings.push(
          <>
            <h3 style={{marginTop:10}}>Controlled Joint</h3>
            <Select
              placeholder='Select Joint'
              value={this.props.jointOrdering[this.state.cachedObjective.indices[0]]}
              onChange={(v)=>{
                let current = [...this.state.cachedObjective.indices];
                current[0] = v
                this.updateIndices(current)
              }}
              style={{ width:'100%'}}>
                {this.getIdxOptions()}
            </Select>
            <h3 style={{marginTop:10}}>Reference Joint</h3>
            <Select
              placeholder='Select Joint'
              value={this.props.jointOrdering[this.state.cachedObjective.indices[1]]}
              onChange={(v)=>{
                let current = [...this.state.cachedObjective.indices];
                current[1] = v
                this.updateIndices(current)
              }}
              style={{ width:'100%'}}>
                {this.getIdxOptions()}
            </Select>
          </>
        )
      } else {
        settings.push(
          <>
            <h3 style={{marginTop:10}}>Controlled Joint</h3>
            <Select
              placeholder='Select Joint'
              value={this.props.jointOrdering[this.state.cachedObjective.indices[0]]}
              onChange={(v)=>{this.updateIndices([v])}}
              style={{ width:'100%'}}>
                {this.getIdxOptions()}
            </Select>
          </>
        )
      }
    }
    return settings;
  }

  getScaleSettings () {
    return (
      <>
        <h3 style={{marginTop:10}}>Motion Scale</h3>
        <Row>
          <Col span={12}>
            <Slider
              min={0}
              max={20}
              step={0.01}
              onChange={this.updateScale}
              value={typeof this.state.cachedObjective.scale === 'number' ? this.state.cachedObjective.scale : 0}
            />
          </Col>
          <Col span={4}>
            <InputNumber
              min={0}
              max={20}
              step={0.01}
              style={{ margin: '0 16px' }}
              value={this.state.cachedObjective.scale}
              onChange={this.updateScale}
            />
          </Col>
        </Row>
      </>)
  }

  render() {
    return (
      <>
        <h3>Name</h3>
        <Input placeholder='Name this Attribute' value={this.state.cachedObjective.tag} onChange={(v)=>this.updateTag(v)}/>
        <div style={{marginTop:10, padding: 10, borderRadius:5, backgroundColor: '#ececec'}}>
          {getObjectivePreview(this.state.cachedObjective,
                               this.props.fixedFrame,
                               this.props.eeFixedJoints,
                               this.props.jointOrdering,
                               this.props.jointNames)}</div>
        <h3 style={{marginTop:10}}>Type</h3>
        <Select
          placeholder="Select the Attribute Type"
          value={defaultObjectiveNames[this.state.cachedObjective.variant]}
          onChange={(v)=>this.updateVariant(v)}
          style={{ width:'100%'}}>
            {this.getVariantOptions()}
        </Select>
        {this.getIndexSettings()}
        {(this.state.cachedObjective.scale !== undefined) ? (
          this.getScaleSettings()
        ) : (<></>)}
        {(this.state.cachedObjective.shape !== undefined) ? (
          <>
            <h3 style={{marginTop:10}}>Motion Shape</h3>
            {[0,1,2].map(idx=>(
              <Row>
                <Col span={12}>
                  <Slider
                    min={0}
                    max={2}
                    step={0.01}
                    onChange={(v)=>this.updateShape(v,idx)}
                    value={typeof this.state.cachedObjective.shape[idx] === 'number' ? this.state.cachedObjective.shape[idx] : 0}
                  />
                </Col>
                <Col span={4}>
                  <InputNumber
                    min={0}
                    max={20}
                    step={0.01}
                    style={{ margin: '0 16px' }}
                    value={this.state.cachedObjective.shape[idx]}
                    onChange={(v)=>this.updateShape(v,idx)}
                  />
                </Col>
              </Row>
            ))}
          </>
        ) : (<></>)}
        {(this.state.cachedObjective.frequency !== undefined) ? (
          <>
          <h3 style={{marginTop:10}}>Motion Length</h3>
          <Row>
            <Col span={12}>
              <Slider
                min={0}
                max={20}
                step={0.01}
                onChange={this.updateFrequency}
                value={typeof this.state.cachedObjective.frequency === 'number' ? this.state.cachedObjective.frequency : 0}
              />
            </Col>
            <Col span={4}>
              <InputNumber
                min={0}
                max={20}
                step={0.01}
                style={{ margin: '0 16px' }}
                value={this.state.cachedObjective.frequency}
                onChange={this.updateFrequency}
              />
            </Col>
          </Row>
          </>
        ) : (<></>)}
        <h3 style={{marginTop:10}}>Strength by Mode</h3>
        <List header={null} footer={null} bordered dataSource={this.props.modeNames.map((mode,idx)=>idx)}
              renderItem={(idx)=>this.getWeightSlider(idx)}
        />
      </>
    )
  }

}

export default ObjectiveSpec
