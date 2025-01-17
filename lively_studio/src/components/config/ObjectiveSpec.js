import React, { useState } from 'react';
import _ from 'lodash';
import { List, Select, Input, Button,
         Row, Cascader, Drawer, Space } from 'antd';
import { getObjectivePreview } from '../../util/Englishify';
import { getObjectiveMarkers } from '../../util/Markers';
import { objectiveMeta } from '../../util/Default';
import ScalarInput from '../../util/ScalarInput';
import { PAIRED_OBJECTIVES,
         CARTESIAN_OBJECTIVES,
         JOINT_OBJECTIVES } from '../../util/Categories';

export default function ObjectiveSpec(props) {

  let visible = props.meta.selected && props.meta.selected.type === 'objective';
  let onFile = visible ? props.config.objectives[props.meta.selected.idx] : {tag: '', variant: '', indices: []};

  const [cachedObjective, setCachedObjective] = useState(onFile);
  const [advancedFreq, setAdvancedFreq] = useState([5,10,15].indexOf(cachedObjective.frequency) >= 0)

  let matches = _.isEqual(cachedObjective, onFile);

  const isJointObjective = (JOINT_OBJECTIVES.indexOf(cachedObjective.variant) >= 0);
  const isCartesianObjective = (CARTESIAN_OBJECTIVES.indexOf(cachedObjective.variant) >= 0);
  const isPairedObjective = (PAIRED_OBJECTIVES.indexOf(cachedObjective.variant) >= 0);

  const cartesianIndicesOptions = props.config.joint_names.map((chain,chainIdx)=>{
    let option = {value:chainIdx,label:`Chain ${chainIdx+1}`,children:chain.map((jointName,jointIdx)=>({value:jointIdx,label:jointName}))};
    option.children.push({value:chain.length,label:props.config.ee_fixed_joints[chainIdx]});
    return option
  });
  const jointIndicesOptions = props.config.joint_ordering.map((jointName,jointIdx)=>({value:jointIdx,label:jointName}));

  return (
    <Drawer
      afterVisibleChange={(visible)=>{
        let onFile = visible ? props.config.objectives[props.meta.selected.idx] : {tag: '', variant: '', indices: []};
        setCachedObjective(onFile)
      }}
      title={
        <Space align="start">
          <h3>{"Specify - "+cachedObjective.tag}</h3>
          {!matches &&
            <Button size="small" type="primary" onClick={()=>{
              let objectives = [...props.config.objectives];
              objectives[props.meta.selected.idx] = cachedObjective;
              let markers = getObjectiveMarkers({objective:cachedObjective,
                                                 goal:props.meta.target_goals[props.meta.selected.idx],
                                                 tree:props.meta.robot_tree,
                                                 poses:props.meta.joint_poses,
                                                 jointNames:props.config.joint_names,
                                                 jointOrdering:props.config.joint_ordering,
                                                 eeFixedJoints:props.config.ee_fixed_joints,
                                                 fixedFrame:props.config.fixed_frame
                                                });
              props.onUpdate({directive:'update',config:{objectives:objectives,
                                                         modes:props.config.modes,
                                                         goals:props.config.goals},
                                                 meta:{gui_markers:markers}})
            }}>
              Save
            </Button>}
          {!matches &&
            <Button size="small" danger type="ghost" onClick={()=>{
              props.onUpdate({directive:'update',meta:{selected:null,gui_markers:{}}})
              setCachedObjective(onFile)
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

        <div style={{marginBottom:10, padding: 10, borderRadius:5, backgroundColor: '#ececec'}}>
          {getObjectivePreview(cachedObjective,
                               props.config.fixed_frame,
                               props.config.ee_fixed_joints,
                               props.config.joint_ordering,
                               props.config.joint_names)}
        </div>

        <h3>Name</h3>
        <Input placeholder='Name this Behavior'
             value={cachedObjective.tag}
             onChange={(e)=>{
               let editedObjective = {...cachedObjective};
               editedObjective.tag = e.target.value
               setCachedObjective(editedObjective)
             }}/>

        {(isCartesianObjective) && (
          <>
            <h3 style={{marginTop:10}}>Cartesian-Controlled Joint</h3>
            <Cascader style={{ width:'100%'}}
                      expandTrigger="hover"
                      value={[cachedObjective.indices[0],cachedObjective.indices[1]]}
                      options={cartesianIndicesOptions}
                      onChange={(v)=>{
                        let editedObjective = {...cachedObjective};
                        let indices = [...editedObjective.indices];
                        indices[0] = v[0]
                        indices[1] = v[1]
                        let defaultName = objectiveMeta[editedObjective.variant].name;
                        let jointName = props.config.joint_names[v[0]][v[1]];
                        if (jointName === undefined) {
                          jointName = props.config.ee_fixed_joints[v[0]]
                        }
                        editedObjective.tag = `${defaultName} - ${jointName}`;
                        editedObjective.indices = indices;
                        setCachedObjective(editedObjective)
                        let markers = getObjectiveMarkers({objective:editedObjective,
                                                           goal:props.meta.target_goals[props.meta.selected.idx],
                                                           tree:props.meta.robot_tree,
                                                           poses:props.meta.joint_poses,
                                                           jointNames:props.config.joint_names,
                                                           jointOrdering:props.config.joint_ordering,
                                                           eeFixedJoints:props.config.ee_fixed_joints,
                                                           fixedFrame:props.config.fixed_frame
                                                          });
                        props.onUpdate({directive:'update',meta:{gui_markers:markers}})
                      }}
            />
            {(isPairedObjective) && (
              <>
                <h3 style={{marginTop:10}}>Reference Joint</h3>
                <Cascader style={{ width:'100%'}}
                          expandTrigger="hover"
                          value={[cachedObjective.indices[2],cachedObjective.indices[3]]}
                          options={cartesianIndicesOptions}
                          onChange={(v)=>{
                            let editedObjective = {...cachedObjective};
                            let indices = [...editedObjective.indices];
                            indices[2] = v[0]
                            indices[3] = v[1]
                            let defaultName = objectiveMeta[editedObjective.variant].name;
                            let jointName = props.config.joint_names[v[0]][v[1]];
                            if (jointName === undefined) {
                              jointName = props.config.ee_fixed_joints[v[0]]
                            }
                            editedObjective.tag = `${defaultName} - ${jointName}`;
                            editedObjective.indices = indices
                            setCachedObjective(editedObjective)
                            let markers = getObjectiveMarkers({objective:editedObjective,
                                                               goal:props.meta.target_goals[props.meta.selected.idx],
                                                               tree:props.meta.robot_tree,
                                                               poses:props.meta.joint_poses,
                                                               jointNames:props.config.joint_names,
                                                               jointOrdering:props.config.joint_ordering,
                                                               eeFixedJoints:props.config.ee_fixed_joints,
                                                               fixedFrame:props.config.fixed_frame
                                                              });
                            props.onUpdate({directive:'update',meta:{gui_markers:markers}})
                          }}/>
              </>
            )}

          </>
        )}

        {(isJointObjective) && (
          <>
            <h3 style={{marginTop:10}}>Controlled Joint</h3>
            <Select
              placeholder='Select Joint'
              value={cachedObjective.indices[0]}
              options={jointIndicesOptions}
              onChange={(v)=>{
                let editedObjective = {...cachedObjective};
                let indices = [...editedObjective.indices];
                indices[0] = v
                let defaultName = objectiveMeta[editedObjective.variant].name;
                editedObjective.tag = `${defaultName} - ${props.config.joint_ordering[indices[0]]}`;
                editedObjective.indices = indices
                setCachedObjective(editedObjective)
                let markers = getObjectiveMarkers({objective:editedObjective,
                                                   goal:props.meta.target_goals[props.meta.selected.idx],
                                                   tree:props.meta.robot_tree,
                                                   poses:props.meta.joint_poses,
                                                   jointNames:props.config.joint_names,
                                                   jointOrdering:props.config.joint_ordering,
                                                   eeFixedJoints:props.config.ee_fixed_joints,
                                                   fixedFrame:props.config.fixed_frame
                                                  });
                props.onUpdate({directive:'update',meta:{gui_markers:markers}})
              }}
              style={{ width:'100%'}}/>
            {(isPairedObjective) && (
              <>
                <h3 style={{marginTop:10}}>Reference Joint</h3>
                <Select
                  placeholder='Select Joint'
                  value={cachedObjective.indices[1]}
                  options={jointIndicesOptions}
                  onChange={(v)=>{
                    let editedObjective = {...cachedObjective};
                    let indices = [...editedObjective.indices];
                    indices[1] = v
                    let defaultName = objectiveMeta[editedObjective.variant].name;
                    editedObjective.tag = `${defaultName} - ${props.config.joint_ordering[indices[1]]}`;
                    editedObjective.indices = indices
                    setCachedObjective(editedObjective)
                    let markers = getObjectiveMarkers({objective:editedObjective,
                                                       goal:props.meta.target_goals[props.meta.selected.idx],
                                                       tree:props.meta.robot_tree,
                                                       poses:props.meta.joint_poses,
                                                       jointNames:props.config.joint_names,
                                                       jointOrdering:props.config.joint_ordering,
                                                       eeFixedJoints:props.config.ee_fixed_joints,
                                                       fixedFrame:props.config.fixed_frame
                                                      });
                    props.onUpdate({directive:'update',meta:{gui_markers:markers}})
                  }}
                  style={{ width:'100%'}}/>
              </>
            )}

          </>
        )}

        {(cachedObjective.scale !== undefined) && (
         <>
          <h3 style={{marginTop:10}}>Motion Scale</h3>
          <Row justify='space-around' style={{marginBottom:10, marginTop: 10, padding: 10, borderRadius:5, backgroundColor: '#ececec'}}>
          <ScalarInput
            step={0.01}
            min={0}
            max={5}
            value={cachedObjective.scale}
            onChange={(v)=>{
              let editedObjective = {...cachedObjective};
              editedObjective.scale = v;
              setCachedObjective(editedObjective);
              let markers = getObjectiveMarkers({objective:editedObjective,
                                                 goal:props.meta.target_goals[props.meta.selected.idx],
                                                 tree:props.meta.robot_tree,
                                                 poses:props.meta.joint_poses,
                                                 jointNames:props.config.joint_names,
                                                 jointOrdering:props.config.joint_ordering,
                                                 eeFixedJoints:props.config.ee_fixed_joints,
                                                 fixedFrame:props.config.fixed_frame
                                                });
              props.onUpdate({directive:'update',meta:{gui_markers:markers}})
            }}
            showInput={true}/>
          </Row>
        </>
        )}

        {(cachedObjective.shape !== undefined) && (
          <>
            <h3 style={{marginTop:10}}>Motion Shape</h3>
            <List header={null} footer={null}
                  bordered dataSource={[0,1,2]}
                  style={{width:'100%'}}
                  renderItem={(dim,idx)=>(
                    <List.Item extra={
                      <ScalarInput
                        step={0.01}
                        min={0}
                        max={cachedObjective.variant === 'orientation_liveliness' ? Math.PI*2 : 2}
                        radians={cachedObjective.variant === 'orientation_liveliness'}
                        value={cachedObjective.shape[idx]}
                        onChange={(v)=>{
                          console.log(v)
                          let editedObjective = {...cachedObjective};
                          let shape = [...editedObjective.shape];
                          shape[idx] = v;
                          editedObjective.shape = shape;
                          setCachedObjective(editedObjective);
                          let markers = getObjectiveMarkers({objective:editedObjective,
                                                             goal:props.meta.target_goals[props.meta.selected.idx],
                                                             tree:props.meta.robot_tree,
                                                             poses:props.meta.joint_poses,
                                                             jointNames:props.config.joint_names,
                                                             jointOrdering:props.config.joint_ordering,
                                                             eeFixedJoints:props.config.ee_fixed_joints,
                                                             fixedFrame:props.config.fixed_frame
                                                            });
                          props.onUpdate({directive:'update',meta:{gui_markers:markers}})
                        }}
                        showInput={true}/>
                      }>
                      <List.Item.Meta title={cachedObjective.variant === 'orientation_liveliness' ? `${['Roll','Pitch','Yaw'][idx]}` : `${['X','Y','Z'][idx]} Dimension`}/>
                    </List.Item>
                  )}
            />
          </>
        )}

        {(cachedObjective.frequency !== undefined) && (
          <>
          <h3 style={{marginTop:10}}>Motion Speed</h3>
          <Row justify='space-around'>
          {[5,10,15].map((val,idx)=>(
            <Button
              type={val === cachedObjective.frequency ? 'primary' : 'ghost'}
              onClick={()=>{
                let editedObjective = {...cachedObjective};
                editedObjective.frequency = val;
                setCachedObjective(editedObjective);
                setAdvancedFreq(false)
              }}>
              {['Slow','Medium','Fast'][idx]}
            </Button>
          ))}
          <Button
            type={advancedFreq ? 'primary' : 'ghost'}
            onClick={()=>{
              setAdvancedFreq(true)
            }}>
            Custom
          </Button>

          </Row>

          {(advancedFreq) && (
            <Row justify='space-around' style={{marginBottom:10, marginTop: 10, padding: 10, borderRadius:5, backgroundColor: '#ececec'}}>
            <ScalarInput
              step={0.01}
              min={-10}
              max={30}
              value={20-cachedObjective.frequency}
              onChange={(v)=>{
                let editedObjective = {...cachedObjective};
                editedObjective.frequency = 20-v;
                setCachedObjective(editedObjective);
              }}
              showInput={true}/>
            </Row>

          )}

        </>)}

      </>)}
    </Drawer>
  )


}
