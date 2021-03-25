import React, { useState } from 'react';
import _ from 'lodash';
import { List, Input, Alert, Drawer, Space, Button } from 'antd';
import TranslationInput from '../../util/TranslationInput';
import RotationInput from '../../util/RotationInput';
import ScalarInput from '../../util/ScalarInput';

export default function GoalSpec(props) {

  let visible = props.meta.selected && props.meta.selected.type === 'goal';
  let onFile = visible ? props.config.goals[props.meta.selected.idx] : {name:'null',values:[]}
  let goalNames = props.config.goals.map(goal=>goal.name);
  let isDefault = props.meta.selected && props.meta.selected.idx === 0;

  const [cachedGoal, setCachedGoal] = useState(onFile);

  let validName = true;
  if (cachedGoal.name !== onFile.name && goalNames.indexOf(cachedGoal.name) >= 0) {
    validName = false
  } else if (!isDefault && cachedGoal.name !== undefined && cachedGoal.name.toLowerCase() === 'default') {
    validName = false
  } else if (cachedGoal.name === '') {
    validName = false
  }

  let matches = _.isEqual(cachedGoal, onFile);

  return (
    <Drawer
      afterVisibleChange={(visible)=>{
        let onFile = visible ? props.config.goals[props.meta.selected.idx] : {name:'null',values:[]}
        setCachedGoal(onFile)
      }}
      title={
        <Space align="start">
          <h3>{isDefault ? "Specify Default Goal" : "Specify Goal - "+cachedGoal.name}</h3>
          {!matches &&
            <Button size="small" type="primary" onClick={()=>{
              let goals = [...props.config.goals];
              goals[props.meta.selected.idx] = cachedGoal;
              props.onUpdate({directive:'update',config:{goals:goals}})
            }}>
              Save
            </Button>}
          {!matches &&
            <Button size="small" danger type="ghost" onClick={()=>{
              props.onUpdate({directive:'update',meta:{selected:null}})
              setCachedGoal(onFile)
            }}>
              Discard
            </Button>
          }
        </Space>
      }
      placement="right"
      closable={matches}
      maskClosable={matches}
      width='50%'
      onClose={()=>props.onUpdate({directive:'update',meta:{selected:null}})}
      visible={visible}
      getContainer={false}
      style={{ position: 'absolute' }}
    >
      {visible && (
        <>

        {!isDefault &&
          <>
          <h3>Name</h3>
          <Input placeholder='Name this Goal'
               value={cachedGoal.name}
               onChange={(e)=>{
                 let editedCache = {...cachedGoal};
                 editedCache.name = e.target.value
                 setCachedGoal(editedCache)
               }}/>
          </>
          }
        {!validName && (
          <Alert
             message="Name Error"
             description="Please select a unique name for this goal."
             type="error"
             showIcon
           />
         )}
        <h3 style={{marginTop:10}}>Direction Values</h3>
        <List header={null} footer={null} style={{}} bordered dataSource={cachedGoal.values.map((value,idx)=>idx)}
              renderItem={(idx)=>{
                if (cachedGoal.values[idx].vector !== undefined) {
                  return (
                    <List.Item key={idx} label={props.config.objectives[idx].tag}>
                      <List.Item.Meta title={props.config.objectives[idx].tag}
                                      description={<TranslationInput
                                                      step={0.01}
                                                      value={cachedGoal.values[idx].vector}
                                                      onChange={(v)=>{
                                                        let editedCache = {...cachedGoal};
                                                        editedCache.values[idx].vector = v;
                                                        props.onUpdate({directive:'update',meta:{target_goals:editedCache.values}});
                                                        setCachedGoal(editedCache);
                                                      }}
                                                      min={[-5,-5,-5]} max={[5,5,5]} showInput={true}/>}/>
                    </List.Item>
                  )
                } else if (cachedGoal.values[idx].quaternion !== undefined) {
                  return (
                    <List.Item key={idx} label={props.config.objectives[idx].tag}>
                      <List.Item.Meta title={props.config.objectives[idx].tag}
                                      description={<RotationInput
                                                      euler={true}
                                                      step={0.1}
                                                      value={cachedGoal.values[idx].quaternion}
                                                      onChange={(v)=>{
                                                        let editedCache = {...cachedGoal};
                                                        editedCache.values[idx].quaternion = v;
                                                        props.onUpdate({directive:'update',meta:{target_goals:editedCache.values}});
                                                        setCachedGoal(editedCache);
                                                      }}
                                                      showInput={true}/>}/>
                    </List.Item>)
                } else if (cachedGoal.values[idx].scalar !== undefined) {
                   return (
                    <List.Item key={idx} label={props.config.objectives[idx].tag}>
                      <List.Item.Meta title={props.config.objectives[idx].tag}
                                      description={<ScalarInput
                                                    step={0.01}
                                                    min={props.config.joint_limits[props.config.objectives[idx].indices[0]][0]}
                                                    max={props.config.joint_limits[props.config.objectives[idx].indices[0]][1]}
                                                    value={cachedGoal.values[idx].scalar}
                                                    onChange={(v)=>{
                                                      let editedCache = {...cachedGoal};
                                                      editedCache.values[idx].scalar = v;
                                                      props.onUpdate({directive:'update',meta:{target_goals:editedCache.values}});
                                                      setCachedGoal(editedCache);
                                                    }}
                                                    showInput={true}/>}/>

                    </List.Item>)
                }
              }}
        />
        </>
      )}
    </Drawer>
  )

}
