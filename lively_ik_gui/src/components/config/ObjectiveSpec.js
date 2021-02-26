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

  debounce = (func, timeout = 200) => {
    let timer;
    return (...args) => {
      clearTimeout(timer);
      timer = setTimeout(() => { func.apply(this, args); }, timeout);
    };
  }

  getVariantOptions = () => {
    let optionNames = [];
    if (BASE_OBJECTIVES.indexOf(this.props.objective.variant) >= 0) {
      optionNames = BASE_OBJECTIVES;
    } else if (DIRECTION_OBJECTIVES.indexOf(this.props.objective.variant) >= 0) {
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
    if (JOINT_OBJECTIVES.indexOf(this.props.objective.variant) >= 0) {
      optionNames = this.props.jointOrdering;
    } else if (CARTESIAN_OBJECTIVES.indexOf(this.props.objective.variant) >= 0) {
      optionNames = this.props.eeFixedJoints;
    }
    return optionNames.map((name,idx)=>(
      <Option value={idx}>{name}</Option>
    ))
  }

  getIdxValue = (idx) => {
    if (JOINT_OBJECTIVES.indexOf(this.props.objective.variant) >= 0) {
      return this.props.jointOrdering[idx];
    } else if (CARTESIAN_OBJECTIVES.indexOf(this.props.objective.variant) >= 0) {
      return this.props.eeFixedJoints[idx];
    }
  }

  updateTag = (e) => {
    let objective = {...this.props.objective};
    objective.tag = e.target.value;
    this.props.onUpdate({objective:objective,modeWeights:this.props.modeWeights})
  }

  getGoalFromCurrent = (variant,idx1,idx2) => {
    switch (variant) {
      case 'position_match':
        let position = this.props.config.joint_poses[idx1][idx2]['position']
        return {vector:[position.x,position.y,position.z]}
      case 'orientation_match':
        let quaternion = this.props.config.joint_poses[idx1][idx2]['quaternion']
        return {quaternion:[quaternion.w,quaternion.x,quaternion.y,quaternion.z]}
      case 'joint_match':
        return {scalar:this.props.displayedState[idx1]}
      default:
        return {}
    }
  }

  updateVariant = (variant) => {
    let currObj = {...this.props.objective};
    let weights = [...this.props.modeWeights];
    let goalValues = [...this.props.goalValues];
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
    this.props.onUpdate({objective:newObj,modeWeights:weights,goalValues:goalValues})
  }

  updateIndices = (values) => {
    let objective = {...this.props.objective};
    let goalValues = [...this.props.goalValues];
    objective.indices = values;

    // Update goal values across all the goals
    goalValues.forEach((value,idx)=>{
      goalValues[idx] = this.getGoalFromCurrent(objective.variant,objective.indices[0],objective.indices[1])
    })

    this.props.onUpdate({objective:objective,goalValues:goalValues})
  }

  updateScale = (value) => {
    let objective = {...this.props.objective};
    objective.scale = value;
    this.props.onUpdate({objective:objective})
  }

  updateShape = (value,idx) => {
    let objective = {...this.props.objective};
    objective.shape[idx] = value;
    this.props.onUpdate({objective:objective})
  }

  updateFrequency = (value) => {
    let objective = {...this.props.objective};
    objective.frequency = value;
    this.props.onUpdate({objective:objective})
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
    if (JOINT_OBJECTIVES.indexOf(this.props.objective.variant) >= 0) {
      return 'Select the joint';
    } else if (CARTESIAN_OBJECTIVES.indexOf(this.props.objective.variant) >= 0) {
      return 'Select the arm\'s end effector';
    }
  }

  updateWeightAtIdx = (idx,value) => {
    let modeWeights = [...this.props.modeWeights];
    modeWeights[idx] = value
    this.props.onUpdate({modeWeights:this.props.modeWeights})
  }

  getWeightSlider = (idx) => {
    return (
      <List.Item key={this.props.modeNames[idx]}>
        <List.Item.Meta title={this.props.modeNames[idx] === 'default' ? 'Default' : this.props.modeNames[idx]}
                        description={<LogSlider min={0} max={100} step={0.01} showInput={true} value={this.props.modeWeights[idx]}
                                                onChange={(v)=>this.debounce(this.updateWeightAtIdx(idx,v))}/>}
        />
      </List.Item>
    )
  }

  getIndexSettings = () => {
    console.log(this.props.objective.indices);
    let settings = [];
    if ((CARTESIAN_OBJECTIVES).indexOf(this.props.objective.variant) >= 0) {
      if (PAIRED_OBJECTIVES.indexOf(this.props.objective.variant) >= 0) {
        settings.push(
          <>
            <h3 style={{marginTop:10}}>Cartesian-Controlled Joint</h3>
            <Cascader style={{ width:'100%'}}
                      expandTrigger="hover"
                      value={[this.props.objective.indices[0],this.props.objective.indices[1]]}
                      options={this.getIdxOptionsFromJointNames()}
                      onChange={(v)=>{
                        let current = [...this.props.objective.indices];
                        current[0] = v[0]
                        current[1] = v[1]
                        this.updateIndices(current)
                      }}
            />
            <h3 style={{marginTop:10}}>Reference Joint</h3>
            <Cascader style={{ width:'100%'}}
                      expandTrigger="hover"
                      value={[this.props.objective.indices[2],this.props.objective.indices[3]]}
                      options={this.getIdxOptionsFromJointNames()}
                      onChange={(v)=>{
                        let current = [...this.props.objective.indices];
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
                      value={this.props.objective.indices}
                      options={this.getIdxOptionsFromJointNames()}
                      onChange={v=>this.updateIndices(v)}
            />
          </>
        )
      }
    } else if ((JOINT_OBJECTIVES).indexOf(this.props.objective.variant) >= 0) {
      if (PAIRED_OBJECTIVES.indexOf(this.props.objective.variant) >= 0) {
        settings.push(
          <>
            <h3 style={{marginTop:10}}>Controlled Joint</h3>
            <Select
              placeholder='Select Joint'
              value={this.props.jointOrdering[this.props.objective.indices[0]]}
              onChange={(v)=>{
                let current = [...this.props.objective.indices];
                current[0] = v
                this.updateIndices(current)
              }}
              style={{ width:'100%'}}>
                {this.getIdxOptions()}
            </Select>
            <h3 style={{marginTop:10}}>Reference Joint</h3>
            <Select
              placeholder='Select Joint'
              value={this.props.jointOrdering[this.props.objective.indices[1]]}
              onChange={(v)=>{
                let current = [...this.props.objective.indices];
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
              value={this.props.jointOrdering[this.props.objective.indices[0]]}
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

  render() {
    console.log(this.props.objective)
    return (
      <>
        <h3>Name</h3>
        <Input placeholder='Name this Attribute' value={this.props.objective.tag} onChange={(v)=>this.debounce(this.updateTag(v))}/>
        <div style={{marginTop:10, padding: 10, borderRadius:5, backgroundColor: '#ececec'}}>
          {getObjectivePreview(this.props.objective,
                               this.props.fixedFrame,
                               this.props.eeFixedJoints,
                               this.props.jointOrdering,
                               this.props.jointNames)}</div>
        <h3 style={{marginTop:10}}>Type</h3>
        <Select
          placeholder="Select the Attribute Type"
          value={defaultObjectiveNames[this.props.objective.variant]}
          onChange={(v)=>this.updateVariant(v)}
          style={{ width:'100%'}}>
            {this.getVariantOptions()}
        </Select>
        {this.getIndexSettings()}
        {(this.props.objective.scale !== undefined) ? (
          <>
          <h3 style={{marginTop:10}}>Motion Scale</h3>
          <Row>
            <Col span={12}>
              <Slider
                min={0}
                max={20}
                step={0.01}
                onChange={this.updateScale}
                value={typeof this.props.objective.scale === 'number' ? this.props.objective.scale : 0}
              />
            </Col>
            <Col span={4}>
              <InputNumber
                min={0}
                max={20}
                step={0.01}
                style={{ margin: '0 16px' }}
                value={this.props.objective.scale}
                onChange={this.updateScale}
              />
            </Col>
          </Row>
          </>
        ) : (<></>)}
        {(this.props.objective.shape !== undefined) ? (
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
                    value={typeof this.props.objective.shape[idx] === 'number' ? this.props.objective.shape[idx] : 0}
                  />
                </Col>
                <Col span={4}>
                  <InputNumber
                    min={0}
                    max={20}
                    step={0.01}
                    style={{ margin: '0 16px' }}
                    value={this.props.objective.shape[idx]}
                    onChange={(v)=>this.updateShape(v,idx)}
                  />
                </Col>
              </Row>
            ))}
          </>
        ) : (<></>)}
        {(this.props.objective.frequency !== undefined) ? (
          <>
          <h3 style={{marginTop:10}}>Motion Speed</h3>
          <Row>
            <Col span={12}>
              <Slider
                min={0}
                max={20}
                step={0.01}
                onChange={this.updateFrequency}
                value={typeof this.props.objective.frequency === 'number' ? this.props.objective.frequency : 0}
              />
            </Col>
            <Col span={4}>
              <InputNumber
                min={0}
                max={20}
                step={0.01}
                style={{ margin: '0 16px' }}
                value={this.props.objective.frequency}
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
