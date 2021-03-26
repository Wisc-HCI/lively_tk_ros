import React, { useState } from 'react';
import { Button, Space, Popover, InputNumber } from 'antd';
import { RedoOutlined, UndoOutlined, EditOutlined } from '@ant-design/icons';
import {eulerFromQuaternion, quaternionFromEuler} from './Geometry';

function RotationInput(props) {

  const [popoverVisible, setPopoverVisible] = useState(false);

  const eulerValues = eulerFromQuaternion(props.value);

  return (
      <Space wrap={true} align='center'>

        <Button shape="circle" onClick={()=>props.onChange(quaternionFromEuler([eulerValues[0]+props.step,eulerValues[1],eulerValues[2]]))} icon={<UndoOutlined style={{color:'red'}}/>} />
        <Button shape="circle" onClick={()=>props.onChange(quaternionFromEuler([eulerValues[0]-props.step,eulerValues[1],eulerValues[2]]))} icon={<RedoOutlined style={{color:'red'}}/>} />
        <Button shape="circle" onClick={()=>props.onChange(quaternionFromEuler([eulerValues[0],eulerValues[1]-props.step,eulerValues[2]]))} icon={<UndoOutlined style={{color:'lime'}}/>} />
        <Button shape="circle" onClick={()=>props.onChange(quaternionFromEuler([eulerValues[0],eulerValues[1]+props.step,eulerValues[2]]))} icon={<RedoOutlined style={{color:'lime'}}/>} />
        <Button shape="circle" onClick={()=>props.onChange(quaternionFromEuler([eulerValues[0],eulerValues[1],eulerValues[2]-props.step]))} icon={<UndoOutlined style={{color:'blue'}}/>} />
        <Button shape="circle" onClick={()=>props.onChange(quaternionFromEuler([eulerValues[0],eulerValues[1],eulerValues[2]+props.step]))} icon={<RedoOutlined style={{color:'blue'}}/>} />


        {props.showInput ?
          <Popover
            placement='left'
            content={
              <Space>
                <h4 style={{color:'red'}}>R</h4>
                <InputNumber
                  min={0}
                  max={2*Math.pi}
                  step={props.step}
                  style={{ margin: '0 16px' }}
                  value={eulerValues[0]}
                  onChange={(v)=>props.onChange(quaternionFromEuler([v,eulerValues[1],eulerValues[2]]))}
                />
                <h4 style={{color:'lime'}}>P</h4>
                <InputNumber
                  min={0}
                  max={2*Math.pi}
                  step={props.step}
                  style={{ margin: '0 16px' }}
                  value={eulerValues[1]}
                  onChange={(v)=>props.onChange(quaternionFromEuler([eulerValues[0],v,eulerValues[2]]))}
                />
                <h4 style={{color:'blue'}}>Y</h4>
                <InputNumber
                  min={0}
                  max={2*Math.pi}
                  step={props.step}
                  style={{ margin: '0 16px' }}
                  value={eulerValues[2]}
                  onChange={(v)=>props.onChange(quaternionFromEuler([eulerValues[0],eulerValues[1],v]))}
                />
              </Space>
            }
            title="Set Rotation"
            trigger="click"
            visible={popoverVisible}
            onVisibleChange={setPopoverVisible}
            >
            <Button shape='circle' type="primary" icon={<EditOutlined/>} onClick={()=>setPopoverVisible(!popoverVisible)}/>
          </Popover>
          : null}
      </Space>
  )
}

export default RotationInput;
