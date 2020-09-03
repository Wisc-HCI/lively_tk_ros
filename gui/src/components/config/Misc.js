import React from 'react';
import { Form, Slider, Button, Popover, Badge } from 'antd';
import { Collapse } from 'antd';
const { Panel } = Collapse;

class Misc extends React.Component {

  render() {
    return (
      <>
        <h5 style={{backgroundColor:'#e8e8e8', borderRadius:3, padding:10}}>
          Miscellaneous configuration parameters.
        </h5>
        <Form>
        </Form>
      </>
    )
  }

}

export default Misc
