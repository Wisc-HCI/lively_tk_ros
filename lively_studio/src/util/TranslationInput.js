import React, { useState, memo } from 'react';
import { Button, Space, Popover, InputNumber, Row, Col } from 'antd';
import {ArrowUpOutlined, ArrowDownOutlined, ArrowLeftOutlined, ArrowRightOutlined, EditOutlined} from '@ant-design/icons';

const clamped = (vector, minVec, maxVec) => {
  return vector.map((v,i)=>{
    if (v<minVec[i]) {
      return minVec[i]
    } else if (v>maxVec[i]) {
      return maxVec[i]
    } else {
      return v
    }})
}


function TranslationInput(props) {

  const [popoverVisible, setPopoverVisible] = useState(false);
  const [inputVec, setInputVec] = useState(props.value);

  return (
    <>
      <Row wrap={true} align='left'>
        <Col>
        <Button shape="circle"
                style={{margin:3}}
                onClick={()=>{
                  let updatedVec = clamped([inputVec[0]-props.step,inputVec[1],inputVec[2]],props.min,props.max);
                  setInputVec(updatedVec)
                  props.onChange(updatedVec)
                }}
                icon={<ArrowUpOutlined style={{color:'red'}} rotate={45}/>} />
        </Col>
        <Col>
        <Button shape="circle"
                style={{margin:3}}
                onClick={()=>{
                  let updatedVec = clamped([inputVec[0],inputVec[1]-props.step,inputVec[2]],props.min,props.max);
                  setInputVec(updatedVec)
                  props.onChange(updatedVec)
                }}
                icon={<ArrowLeftOutlined style={{color:'lime'}}  />} />
        </Col>
        <Col>
        <Button shape="circle"
                style={{margin:3}}
                onClick={()=>{
                  let updatedVec = clamped([inputVec[0],inputVec[1],inputVec[2]+props.step],props.min,props.max);
                  setInputVec(updatedVec)
                  props.onChange(updatedVec)
                }}
                icon={<ArrowUpOutlined style={{color:'blue'}} />} />
        </Col>
        <Col>
        {props.showInput &&
          <Popover
            placement='left'
            content={
              <Space>
                <h4 style={{color:'red'}}>X</h4>
                <InputNumber
                  min={props.min[0]}
                  max={props.max[0]}
                  step={props.step}
                  precision={2}
                  style={{ margin: '0 16px' }}
                  defaultValue={inputVec[0]}
                  onChange={(v)=>{if (typeof v === 'number') {
                    let updatedVec = [v,inputVec[1],inputVec[2]];
                    setInputVec(updatedVec)
                    props.onChange(updatedVec)
                  }}}
                />
                <h4 style={{color:'lime'}}>Y</h4>
                <InputNumber
                  min={props.min[1]}
                  max={props.max[1]}
                  step={props.step}
                  precision={2}
                  style={{ margin: '0 16px' }}
                  defaultValue={inputVec[1]}
                  onChange={(v)=>{if (typeof v === 'number') {
                    let updatedVec = [inputVec[0],v,inputVec[2]]
                    setInputVec(updatedVec)
                    props.onChange(updatedVec)
                  }}}
                />
                <h4 style={{color:'blue'}}>Z</h4>
                <InputNumber
                  min={props.min[2]}
                  max={props.max[2]}
                  step={props.step}
                  precision={2}
                  style={{ margin: '0 16px' }}
                  defaultValue={inputVec[2]}
                  onChange={(v)=>{if (typeof v === 'number') {
                    let updatedVec = [inputVec[0],inputVec[1],v];
                    setInputVec(updatedVec)
                    props.onChange(updatedVec)
                  }}}
                />
              </Space>
            }
            title="Set Position"
            trigger="click"
            visible={popoverVisible}
            onVisibleChange={setPopoverVisible}
            >
            <Button style={{margin:3}} shape='circle' type="primary" icon={<EditOutlined/>} onClick={()=>setPopoverVisible(!popoverVisible)}/>
          </Popover>
        }
        </Col>


      </Row>
      <Row>
        <Col>
        <Button shape="circle"
                style={{margin:3}}
                onClick={()=>{
                  let updatedVec = clamped([inputVec[0]+props.step,inputVec[1],inputVec[2]],props.min,props.max);
                  setInputVec(updatedVec)
                  props.onChange(updatedVec)
                }}
                icon={<ArrowDownOutlined style={{color:'red'}} rotate={45}/>} />
        </Col>
        <Col>
        <Button shape="circle"
                style={{margin:3}}
                onClick={()=>{
                  let updatedVec = clamped([inputVec[0],inputVec[1]+props.step,inputVec[2]],props.min,props.max);
                  setInputVec(updatedVec)
                  props.onChange(updatedVec)
                }}
                icon={<ArrowRightOutlined style={{color:'lime'}} />} />
        </Col>
        <Col>
        <Button shape="circle"
                style={{margin:3}}
                onClick={()=>{
                  let updatedVec = clamped([inputVec[0],inputVec[1],inputVec[2]-props.step],props.min,props.max);
                  setInputVec(updatedVec)
                  props.onChange(updatedVec)
                }}
                icon={<ArrowDownOutlined style={{color:'blue'}}  />} />
        </Col>
      </Row>
    </>
  )
}

export default memo(TranslationInput);
