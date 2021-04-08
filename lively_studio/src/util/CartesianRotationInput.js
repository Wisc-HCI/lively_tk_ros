import React, { useState } from 'react';
import { Button, Space, Popover, InputNumber } from 'antd';
import {RedoOutlined, UndoOutlined, EditOutlined, ArrowLeftOutlined, ArrowRightOutlined, ArrowUpOutlined, ArrowDownOutlined} from '@ant-design/icons';
import {eulerFromQuaternion, quaternionFromEuler} from './Geometry';
import Quaternion from 'quaternion';

function RotationInput(props) {

  const [popoverVisible, setPopoverVisible] = useState(false);

  // Get the component in each of the directions
  let currentValue = new Quaternion(props.value);
  let currentDirection = currentValue.rotateVector([0,0,1]);

  let simpleRotation = Quaternion.fromAxisAngle(currentDirection,0.0);

  console.log(currentDirection);
  console.log(simpleRotation);

  // Calculate the remainder between the direction and the original
  let remainder = Quaternion.fromBetweenVectors(currentValue, simpleRotation);

  console.log(remainder);

  let up = Quaternion(1,0,0,0).add(remainder);
  let down = Quaternion(0,0,0.707,0.707).add(remainder);

  let eulerValues = eulerFromQuaternion(props.value);

  console.log(up);
  console.log(down);
  console.log(currentValue);
  console.log(currentValue.slerp(up)(0.1))

  return (
      <Space wrap={true} align='center'>

        <Button shape="circle" onClick={()=>props.onChange(currentValue.slerp(up)(0.1).toVector())} icon={<UndoOutlined style={{color:'red'}}/>} />
        <Button shape="circle" onClick={()=>props.onChange(currentValue.slerp(down)(0.1).toVector())} icon={<RedoOutlined style={{color:'red'}}/>} />


        {props.showInput ?
          <Popover
            placement='left'
            content={
              <Space>
                <h4 style={{color:'red'}}>R</h4>
                <InputNumber
                  min={0}
                  max={2*Math.PI}
                  step={props.step}
                  style={{ margin: '0 16px' }}
                  value={eulerValues[0]}
                  onChange={(v)=>props.onChange(quaternionFromEuler([v,eulerValues[1],eulerValues[2]]))}
                />
                <h4 style={{color:'lime'}}>P</h4>
                <InputNumber
                  min={0}
                  max={2*Math.PI}
                  step={props.step}
                  style={{ margin: '0 16px' }}
                  value={eulerValues[1]}
                  onChange={(v)=>props.onChange(quaternionFromEuler([eulerValues[0],v,eulerValues[2]]))}
                />
                <h4 style={{color:'blue'}}>Y</h4>
                <InputNumber
                  min={0}
                  max={2*Math.PI}
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
