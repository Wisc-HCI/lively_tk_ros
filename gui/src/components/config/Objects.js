import React from 'react';
import { Form, Slider, Button, Popover, Badge } from 'antd';
import { Collapse } from 'antd';
const { Panel } = Collapse;

class Objects extends React.Component {

  render() {
    return (
      <>
        <h5 style={{backgroundColor:'#e8e8e8', borderRadius:3, padding:10}}>
          Specify objects in the scene that can collide with the robot.
        </h5>
        <Form>
        </Form>
      </>
    )
  }

}

export default Objects
