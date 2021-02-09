import React from 'react';
import { List, Slider, Select, Input, InputNumber, Row, Col } from 'antd';
import { defaultObjectives, defaultGoals, defaultObjectiveNames } from '../../util/Default';
import { getObjectivePreview } from '../../util/Englishify';
const { Option } = Select;

const DEFAULT_OBJECTIVES = ['joint_limits','nn_collision','env_collision','min_velocity','min_acceleration','min_jerk'];
const DIRECTION_OBJECTIVES = ['ee_position_match','ee_orientation_match','ee_position_mirroring','ee_orientation_mirroring',
                              'ee_position_bounding','ee_orientation_bounding','joint_mirroring','joint_match'];
const LIVELINESS_OBJECTIVES = ['ee_position_liveliness','ee_orientation_liveliness','joint_liveliness','base_link_position_liveliness'];
const EE_OBJECTIVES = ['ee_position_match','ee_orientation_match','ee_position_mirroring','ee_orientation_mirroring',
                       'ee_position_bounding','ee_orientation_bounding','ee_position_liveliness','ee_orientation_liveliness'];
const JOINT_OBJECTIVES = ['joint_mirroring','joint_match','joint_liveliness']

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
    if (DEFAULT_OBJECTIVES.indexOf(this.props.objective.variant) >= 0) {
      optionNames = DEFAULT_OBJECTIVES;
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
    } else if (EE_OBJECTIVES.indexOf(this.props.objective.variant) >= 0) {
      optionNames = this.props.eeFixedJoints;
    }
    return optionNames.map((name,idx)=>(
      <Option value={idx}>{name}</Option>
    ))
  }

  getIdxValue = (idx) => {
    if (JOINT_OBJECTIVES.indexOf(this.props.objective.variant) >= 0) {
      return this.props.jointOrdering[idx];
    } else if (EE_OBJECTIVES.indexOf(this.props.objective.variant) >= 0) {
      return this.props.eeFixedJoints[idx];
    }
  }

  updateTag = (e) => {
    let objective = {...this.props.objective};
    objective.tag = e.target.value;
    this.props.onUpdate({objective:objective,goals:this.props.goals})
  }

  updateVariant = (variant) => {
    let objective = {...this.props.objective};
    let goals = [...this.props.goals];
    let newObj = JSON.parse(JSON.stringify(defaultObjectives[variant]))

    Object.keys(newObj).forEach(field=>{
      if (objective[field]) {
        newObj[field] = objective[field]
      }
    })
    goals.forEach((goal,idx)=>{
      let defaultGoal = JSON.parse(JSON.stringify(defaultGoals[variant]))
      Object.keys(defaultGoal).forEach(field=>{
        if (goal[field]) {
          goals[idx][field] = goal[field]
        }
      })
    })
    this.props.onUpdate({objective:newObj,goals:goals})
  }

  updatePrimaryIdx = (idx) => {
    let objective = {...this.props.objective};
    objective.index = idx;
    this.props.onUpdate({objective:objective,goals:this.props.goals})
  }

  updateSecondaryIdx = (idx) => {
    let objective = {...this.props.objective};
    objective.secondary_index = idx;
    this.props.onUpdate({objective:objective,goals:this.props.goals})
  }

  updateScale = (value) => {
    let objective = {...this.props.objective};
    objective.scale = value;
    this.props.onUpdate({objective:objective,goals:this.props.goals})
  }

  updateFrequency = (value) => {
    let objective = {...this.props.objective};
    objective.frequency = value;
    this.props.onUpdate({objective:objective,goals:this.props.goals})
  }

  getIdxHeader = (primary) => {
    let term = '';
    if (this.props.objective.secondary_index && primary) {
      term = 'Primary ';
    } else if (this.props.objective.secondary_index && !primary) {
      term = 'Secondary ';
    }
    if (JOINT_OBJECTIVES.indexOf(this.props.objective.variant) >= 0) {
      return term+'Joint';
    } else if (EE_OBJECTIVES.indexOf(this.props.objective.variant) >= 0) {
      return term+'Arm End Effector';
    }
  }

  getIdxPlaceholder = () => {
    if (JOINT_OBJECTIVES.indexOf(this.props.objective.variant) >= 0) {
      return 'Select the joint';
    } else if (EE_OBJECTIVES.indexOf(this.props.objective.variant) >= 0) {
      return 'Select the arm\'s end effector';
    }
  }

  render() {
    console.log(this.props.objective)
    return (
      <>
        <h3>Name</h3>
        <Input placeholder='Name this behavior' value={this.props.objective.tag} onChange={(v)=>this.debounce(this.updateTag(v))}/>
        <div style={{marginTop:10, padding: 10, borderRadius:5, backgroundColor: '#ececec'}}>{getObjectivePreview(this.props.objective, this.props.fixedFrame, this.props.eeFixedJoints, this.props.jointOrdering)}</div>
        <h3 style={{marginTop:10}}>Type</h3>
        <Select
          placeholder="Select the Behavior Type"
          value={defaultObjectiveNames[this.props.objective.variant]}
          onChange={(v)=>this.updateVariant(v)}
          style={{ width:'100%'}}>
            {this.getVariantOptions()}
        </Select>
        {(this.props.objective.index !== undefined) ? (
          <>
          <h3 style={{marginTop:10}}>{this.getIdxHeader(true)}</h3>
          <Select
            placeholder={this.getIdxPlaceholder()}
            value={this.getIdxValue(this.props.objective.index)}
            onChange={(v)=>this.updatePrimaryIdx(v)}
            style={{ width:'100%'}}>
              {this.getIdxOptions()}
          </Select>
          </>
        ) : (<></>)}
        {(this.props.objective.secondary_index !== undefined) ? (
          <>
          <h3 style={{marginTop:10}}>{this.getIdxHeader(false)}</h3>
          <Select
            placeholder={this.getIdxPlaceholder()}
            value={this.getIdxValue(this.props.objective.secondary_index)}
            onChange={(v)=>this.updateSecondaryIdx(v)}
            style={{ width:'100%'}}>
              {this.getIdxOptions()}
          </Select>
          </>
        ) : (<></>)}
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
        {(this.props.objective.scale !== undefined) ? (
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
      </>
    )
  }

}

export default ObjectiveSpec
