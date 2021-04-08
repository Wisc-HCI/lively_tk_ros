import React, { useState, memo } from 'react';
import { Button, Space, Popover, InputNumber, Row, Col } from 'antd';
import { RedoOutlined, UndoOutlined, EditOutlined } from '@ant-design/icons';
import {eulerFromQuaternion, quaternionFromEuler} from './Geometry';

const RAD_2_DEG = 180 / Math.PI;
const DEG_2_RAD = Math.PI / 180;

const eulerVecToDegrees = (vec) => {
  return vec.map(v=>RAD_2_DEG*v)
}
const eulerVecToRadians = (vec) => {
  return vec.map(v=>DEG_2_RAD*v)
}

function RotationInput(props) {

  const [popoverVisible, setPopoverVisible] = useState(false);
  const [eulerValues, setEulerValues] = useState(eulerVecToDegrees(eulerFromQuaternion(props.value)));
  const DEGREE_STEP = props.step ? props.step*DEG_2_RAD : 1;

  let limits = [-360,360];
  if (props.onlyPositive) {
    limits[0] = 0
  }

  return (
    <>
      <Row align='left'>
        <Col>
          <Button shape="circle" style={{margin:3}} onClick={()=>{
            if (eulerValues[0]+DEGREE_STEP < limits[1]) {
              let updatedEulerValues = [eulerValues[0]+DEGREE_STEP,eulerValues[1],eulerValues[2]];
              setEulerValues(updatedEulerValues);
              props.onChange(quaternionFromEuler(eulerVecToRadians(updatedEulerValues)));
            }
          }} icon={<UndoOutlined style={{color:'red'}}/>} />
        </Col>
        <Col>
        <Button shape="circle" style={{margin:3}} onClick={()=>{
          if (eulerValues[1]-DEGREE_STEP > limits[0]) {
            let updatedEulerValues = [eulerValues[0],eulerValues[1]-DEGREE_STEP,eulerValues[2]];
            setEulerValues(updatedEulerValues);
            props.onChange(quaternionFromEuler(eulerVecToRadians(updatedEulerValues)));
          }
        }} icon={<UndoOutlined style={{color:'lime'}}/>} />
        </Col>
        <Col>
        <Button shape="circle" style={{margin:3}} onClick={()=>{
          if (eulerValues[2]-DEGREE_STEP > limits[0]) {
            let updatedEulerValues = [eulerValues[0],eulerValues[1],eulerValues[2]-DEGREE_STEP];
            setEulerValues(updatedEulerValues);
            props.onChange(quaternionFromEuler(eulerVecToRadians(updatedEulerValues)));
          }
        }} icon={<UndoOutlined style={{color:'blue'}}/>} />
        </Col>
        <Col>
        {props.showInput &&
          <Popover
            placement='left'
            content={
              <Space>
                <h4 style={{color:'red'}}>R</h4>
                <InputNumber
                  min={limits[0]}
                  max={limits[1]}
                  step={DEGREE_STEP}
                  precision={2}
                  style={{ margin: '0 16px' }}
                  defaultValue={eulerValues[0]}
                  onChange={(v)=>{if (typeof v === 'number') {
                    let updatedEulerValues = [v,eulerValues[1],eulerValues[2]];
                    setEulerValues(updatedEulerValues);
                    props.onChange(quaternionFromEuler(eulerVecToRadians(updatedEulerValues)));
                  }}}
                />
                <h4 style={{color:'lime'}}>P</h4>
                <InputNumber
                  min={limits[0]}
                  max={limits[1]}
                  step={DEGREE_STEP}
                  precision={2}
                  style={{ margin: '0 16px' }}
                  defaultValue={eulerValues[1]}
                  onChange={(v)=>{if (typeof v === 'number') {
                    let updatedEulerValues = [eulerValues[0],v,eulerValues[2]];
                    setEulerValues(updatedEulerValues);
                    props.onChange(quaternionFromEuler(eulerVecToRadians(updatedEulerValues)));
                  }}}
                />
                <h4 style={{color:'blue'}}>Y</h4>
                <InputNumber
                  min={limits[0]}
                  max={limits[1]}
                  step={DEGREE_STEP}
                  precision={2}
                  style={{ margin: '0 16px' }}
                  defaultValue={eulerValues[2]}
                  onChange={(v)=>{if (typeof v === 'number') {
                    let updatedEulerValues = [eulerValues[0],eulerValues[1],v];
                    setEulerValues(updatedEulerValues);
                    props.onChange(quaternionFromEuler(eulerVecToRadians(updatedEulerValues)));
                  }}}
                />
              </Space>
            }
            title="Set Rotation"
            trigger="click"
            visible={popoverVisible}
            onVisibleChange={setPopoverVisible}
            >
            <Button style={{margin:3}} shape='circle' type="primary" icon={<EditOutlined/>} onClick={()=>setPopoverVisible(!popoverVisible)}/>
          </Popover>
        }
        </Col>
      </Row>
      <Row align='left'>
        <Col>
        <Button shape="circle" style={{margin:3}} onClick={()=>{
          if (eulerValues[0]-DEGREE_STEP > limits[0]) {
            let updatedEulerValues = [eulerValues[0]-DEGREE_STEP,eulerValues[1],eulerValues[2]];
            setEulerValues(updatedEulerValues);
            props.onChange(quaternionFromEuler(eulerVecToRadians(updatedEulerValues)));
          }
        }} icon={<RedoOutlined style={{color:'red'}}/>} />
        </Col>
        <Col>
        <Button shape="circle" style={{margin:3}} onClick={()=>{
          if (eulerValues[1]+DEGREE_STEP < limits[1]) {
            let updatedEulerValues = [eulerValues[0],eulerValues[1]+DEGREE_STEP,eulerValues[2]];
            setEulerValues(updatedEulerValues);
            props.onChange(quaternionFromEuler(eulerVecToRadians(updatedEulerValues)));
          }
        }} icon={<RedoOutlined style={{color:'lime'}}/>} />
        </Col>
        <Col>
        <Button shape="circle" style={{margin:3}} onClick={()=>{
          if (eulerValues[2]+DEGREE_STEP < limits[1]) {
            let updatedEulerValues = [eulerValues[0],eulerValues[1],eulerValues[2]+DEGREE_STEP];
            setEulerValues(updatedEulerValues);
            props.onChange(quaternionFromEuler(eulerVecToRadians(updatedEulerValues)));
          }
        }} icon={<RedoOutlined style={{color:'blue'}}/>} />
        </Col>
        <Col>
        </Col>
      </Row>

    </>
  )
}

export default memo(RotationInput);
