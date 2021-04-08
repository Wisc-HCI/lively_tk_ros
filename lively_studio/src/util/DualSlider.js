import React, { useState } from 'react';
import { Slider, InputNumber, Row, Col, Button, Popover, Space } from 'antd';


function DualSlider(props) {

  const [popoverVisible, setPopoverVisible] = useState(false);

  if (props.showInput) {
    return (
      <Row align='middle'>
        <Col span={16}>
          <Slider range={{ draggableTrack: true }} min={props.min} max={props.max}
                  value={props.value} step={props.step} onChange={props.onChange}/>
        </Col>

        <Popover
          content={
            <>
            <Space>
              <h4>Min</h4>
              <InputNumber
                min={props.min}
                max={props.value[1]}
                step={props.step}
                style={{ margin: '0 16px' }}
                value={props.value[0]}
                onChange={(v)=>props.onChange([v,props.value[1]],0)}
              />
            </Space>
            <Space>
              <h4>Max</h4>
              <InputNumber
                min={props.value[0]}
                max={props.max}
                step={props.step}
                style={{ margin: '0 16px' }}
                value={props.value[1]}
                onChange={(v)=>props.onChange([props.value[0],v],0)}
              />
            </Space>
            </>
          }
          title="Set Range"
          trigger="click"
          visible={popoverVisible}
          onVisibleChange={setPopoverVisible}
          >
          <Button size='small' type="primary" onClick={()=>setPopoverVisible(!popoverVisible)}>Edit Manually</Button>
        </Popover>
      </Row>
    )
  } else {
    return (
      <Slider range={{ draggableTrack: true }} min={props.min} max={props.max} value={props.value} step={props.step} onChange={props.onChange} />
    )
  }
}

export default DualSlider;
