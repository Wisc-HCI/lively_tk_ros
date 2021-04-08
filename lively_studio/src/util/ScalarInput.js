import React, { useState, memo } from 'react';
import { Button, Space, Popover, InputNumber } from 'antd';
import {ArrowLeftOutlined, ArrowRightOutlined, EditOutlined} from '@ant-design/icons';

const RAD_2_DEG = 180 / Math.PI;
const DEG_2_RAD = Math.PI / 180;

const clamped = (v, minimum, maximum) => {
  if (v<minimum) {
      return minimum
  } else if (v>maximum) {
      return maximum
  } else {
      return v
  }
}

function ScalarInput(props) {

  const [popoverVisible, setPopoverVisible] = useState(false);
  const step = props.radians ? 1 : props.step;
  const value = +(props.radians ? RAD_2_DEG * props.value : props.value).toFixed(2);
  const min = props.radians ? RAD_2_DEG * props.min : props.min;
  const max = props.radians ? RAD_2_DEG * props.max : props.max;
  const down = props.radians ? props.value - DEG_2_RAD : props.value - props.step;
  const up = props.radians ? props.value + DEG_2_RAD : props.value - props.step;

  return (
      <Space wrap={true} align='center'>
        <Button shape="circle"
                disabled={value <= min}
                onClick={()=>props.onChange(clamped(down,props.min,props.max))}
                icon={<ArrowLeftOutlined/>}/>
        <span>{value}</span>
        <Button shape="circle"
                disabled={value >= max}
                onClick={()=>props.onChange(clamped(up,props.min,props.max))}
                icon={<ArrowRightOutlined/>}/>

        {props.showInput ?
          <Popover
            placement='left'
            content={
              <InputNumber
                  min={min}
                  max={max}
                  step={step}
                  precision={2}
                  style={{ margin: '0 16px' }}
                  defaultValue={value}
                  onChange={(v)=>{if (typeof v === 'number') {if (props.radians) {props.onChange(v * DEG_2_RAD)} else {props.onChange(v)}}}}
                />
            }
            title="Set Value"
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

export default memo(ScalarInput);
