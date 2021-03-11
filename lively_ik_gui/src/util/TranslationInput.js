import React, { useState } from 'react';
import { Button, Space, Popover, InputNumber } from 'antd';
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

  return (
      <Space wrap={true} align='center'>
        <Button shape="circle"
                onClick={()=>props.onChange(clamped([props.value[0]-props.step,props.value[1],props.value[2]],props.min,props.max))}
                icon={<ArrowUpOutlined style={{color:'red'}} rotate={45}/>} />
        <Button shape="circle"
                onClick={()=>props.onChange(clamped([props.value[0]+props.step,props.value[1],props.value[2]],props.min,props.max))}
                icon={<ArrowDownOutlined style={{color:'red'}} rotate={45}/>} />
        <Button shape="circle"
                onClick={()=>props.onChange(clamped([props.value[0],props.value[1]-props.step,props.value[2]],props.min,props.max))}
                icon={<ArrowLeftOutlined style={{color:'lime'}}  />} />
        <Button shape="circle"
                onClick={()=>props.onChange(clamped([props.value[0],props.value[1]+props.step,props.value[2]],props.min,props.max))}
                icon={<ArrowRightOutlined style={{color:'lime'}} />} />
        <Button shape="circle"
                onClick={()=>props.onChange(clamped([props.value[0],props.value[1],props.value[2]+props.step],props.min,props.max))}
                icon={<ArrowUpOutlined style={{color:'blue'}} />} />
        <Button shape="circle"
                onClick={()=>props.onChange(clamped([props.value[0],props.value[1],props.value[2]-props.step],props.min,props.max))}
                icon={<ArrowDownOutlined style={{color:'blue'}}  />} />

        {props.showInput ?
          <Popover
            placement='left'
            content={
              <Space>
                <h4 style={{color:'red'}}>X</h4>
                <InputNumber
                  min={props.min[0]}
                  max={props.max[0]}
                  step={props.step}
                  style={{ margin: '0 16px' }}
                  value={props.value[0]}
                  onChange={(v)=>props.onChange([v,props.value[1],props.value[2]])}
                />
                <h4 style={{color:'lime'}}>Y</h4>
                <InputNumber
                  min={props.min[1]}
                  max={props.max[1]}
                  step={props.step}
                  style={{ margin: '0 16px' }}
                  value={props.value[1]}
                  onChange={(v)=>props.onChange([props.value[0],v,props.value[2]])}
                />
                <h4 style={{color:'blue'}}>Z</h4>
                <InputNumber
                  min={props.min[2]}
                  max={props.max[2]}
                  step={props.step}
                  style={{ margin: '0 16px' }}
                  value={props.value[2]}
                  onChange={(v)=>props.onChange([props.value[0],props.value[1],v])}
                />
              </Space>
            }
            title="Set Position"
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

export default TranslationInput;
