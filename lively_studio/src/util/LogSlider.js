import React from 'react';
import { Slider, InputNumber, Row, Col } from 'antd';


function LogSlider(props) {

  if (props.showInput) {
    return (
      <Row>
        <Col span={12}>
          <Slider
            min={Math.log1p(props.min)}
            max={Math.log1p(props.max)}
            step={props.step}
            tipFormatter={(value)=>(+Math.pow(Math.E,value)-1).toFixed(1)}
            value={Math.log1p(props.value)}
            onChange={(value)=>props.onChange(+(Math.pow(Math.E,value)-1).toFixed(1))}
          />
        </Col>
        <Col span={4}>
          <InputNumber
            min={props.min}
            max={props.max}
            step={props.step}
            style={{ margin: '0 16px' }}
            value={props.value}
            onChange={props.onChange}
          />
        </Col>
      </Row>
    )
  } else {
    return (
      <Slider
        min={Math.log10(props.min)}
        max={Math.log10(props.max)}
        step={props.step}
        tipFormatter={(value)=>Math.pow(value,10)}
        value={Math.log10(props.value)}
        onChange={(value)=>props.onChange(Math.pow(value,10))}
      />
    )
  }
}

export default LogSlider;
