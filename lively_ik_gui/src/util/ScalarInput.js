import React, { useState } from 'react';
import { Button, Space, Popover, InputNumber } from 'antd';
import {ArrowLeftOutlined, ArrowRightOutlined, EditOutlined} from '@ant-design/icons';

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

  return (
      <Space wrap={true} align='center'>
        <Button shape="circle"
                onClick={()=>props.onChange(clamped(props.value-props.step,props.min,props.max))}
                icon={<ArrowLeftOutlined/>}/>
        <Button shape="circle"
                onClick={()=>props.onChange(clamped(props.value+props.step,props.min,props.max))}
                icon={<ArrowRightOutlined/>}/>

        {props.showInput ?
          <Popover
            placement='left'
            content={
              <InputNumber
                  min={props.min}
                  max={props.max}
                  step={props.step}
                  style={{ margin: '0 16px' }}
                  value={props.value}
                  onChange={props.onChange}
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

export default ScalarInput;
