import React, { useState } from 'react';
import _ from 'lodash';
import { List, Input, Alert, Drawer, Space, Button, Tabs, Dropdown, Menu } from 'antd';
import ScalarInput from '../../util/ScalarInput';
import { BEHAVIOR_ATTRIBUTE_GROUPS,
         BEHAVIOR_ATTRIBUTE_GROUP_NAMES } from '../../util/Categories';
const { TabPane } = Tabs;

const debounce = (func, timeout = 200) => {
  let timer;
  return (...args) => {
    clearTimeout(timer);
    timer = setTimeout(() => { func.apply(this, args); }, timeout);
  };
}

export default function ModeSpec(props) {

  let visible = props.meta.selected && props.meta.selected.type === 'mode';
  let onFile = visible ? props.config.modes[props.meta.selected.idx] : {name:'null',weights:[]}
  let modeNames = props.config.modes.map(mode=>mode.name);
  let isDefault = props.meta.selected && props.meta.selected.idx === 0;

  const [cachedMode, setCachedMode] = useState(onFile);
  const [activeTab, setActiveTab] = useState('Base');

  let validName = true;
  if (cachedMode.name !== onFile.name && modeNames.indexOf(cachedMode.name) >= 0) {
    validName = false
  } else if (!isDefault && cachedMode.name !== undefined && cachedMode.name.toLowerCase() === 'default') {
    validName = false
  } else if (cachedMode.name === '') {
    validName = false
  }

  let matches = _.isEqual(cachedMode, onFile);

  return (
    <Drawer
      afterVisibleChange={(visible)=>{
        let onFile = visible ? props.config.modes[props.meta.selected.idx] : {name:'null',weights:[]}
        setCachedMode(onFile)
      }}
      title={
        <Space align="start">
          <h3>{isDefault ? "Specify Default Mode" : "Specify Mode - "+cachedMode.name}</h3>
          {!matches &&
            <Button size="small" type="primary" onClick={()=>{
              let modes = [...props.config.modes];
              modes[props.meta.selected.idx] = cachedMode;
              props.onUpdate({directive:'update',config:{modes:modes}})
            }}>
              Save
            </Button>}
          {!matches &&
            <Button size="small" danger type="ghost" onClick={()=>{
              props.onUpdate({directive:'update',meta:{selected:null}})
              setCachedMode(onFile)
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
          <Input placeholder='Name this Mode'
               value={cachedMode.name}
               onChange={(e)=>{
                 let editedCache = {...cachedMode};
                 editedCache.name = e.target.value
                 setCachedMode(editedCache)
               }}/>
          </>
          }
        {!validName && (
          <Alert
             message="Name Error"
             description="Please select a unique name for this mode."
             type="error"
             showIcon
           />
         )}
        <h3 style={{marginTop:10}}>Behavior Importance</h3>
        <Tabs activeKey={activeTab}
              centered
              style={{height: '100%', width:'100%' }}
              onChange={(key)=>setActiveTab(key)}>
          {BEHAVIOR_ATTRIBUTE_GROUPS.map((group,groupIdx)=>(
            <TabPane tab={BEHAVIOR_ATTRIBUTE_GROUP_NAMES[groupIdx]} key={BEHAVIOR_ATTRIBUTE_GROUP_NAMES[groupIdx]} style={{ height: '100%', width:'100%' }}>
              <List header={null}
                    footer={null}
                    bordered
                    pagination={{
                      onChange: page => {},
                      pageSize: 7,
                    }}
                    dataSource={props.config.objectives.map((obj,idx)=>idx).filter((idx)=>BEHAVIOR_ATTRIBUTE_GROUPS[groupIdx].indexOf(props.config.objectives[idx].variant)>=0)}
                    renderItem={(idx)=>(
                      <List.Item
                        key={idx}
                        label={props.config.objectives[idx].tag}
                        extra={
                          <Dropdown
                            overlay={
                              <Menu onClick={({ key })=>{
                                console.log(key);
                                let editedCache = {...cachedMode};
                                let weights = [...editedCache.weights];
                                let otherModeWeight = props.config.modes[key].weights[idx];
                                weights[idx] = otherModeWeight;
                                editedCache.weights = weights;
                                props.onUpdate({directive:'update',meta:{target_weights:editedCache.weights}});
                                setCachedMode(editedCache);
                              }}>
                                {props.config.modes.map((mode,modeIdx)=>(
                                  <Menu.Item key={modeIdx}>
                                    {mode.name === 'default' ? 'Default' : mode.name}
                                  </Menu.Item>
                                ))}
                              </Menu>
                            }
                            trigger='click'
                            placement="bottomLeft">
                            <Button>Copy From...</Button>
                          </Dropdown>
                        }>
                        <List.Item.Meta title={props.config.objectives[idx].tag}
                                        description={<ScalarInput
                                                        min={0} max={500}
                                                        step={0.01}
                                                        showInput={true}
                                                        value={cachedMode.weights[idx]}
                                                        onChange={(v)=>{
                                                          let editedCache = {...cachedMode};
                                                          editedCache.weights[idx] = v;
                                                          debounce(props.onUpdate({directive:'update',meta:{target_weights:editedCache.weights}}));
                                                          setCachedMode(editedCache);
                                                        }}/>}
                        />
                      </List.Item>
                    )}
              />
            </TabPane>
          ))}
        </Tabs>
      </>
      )}
    </Drawer>
  )

}
