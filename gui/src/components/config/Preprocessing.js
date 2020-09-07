import React from 'react';
import { Progress, Card } from 'antd';

class Preprocessing extends React.Component {

  render() {
    return (
      <>
        <h5 style={{backgroundColor:'#e8e8e8', borderRadius:3, padding:10}}>
          Complete the configuration by preprocessing.
        </h5>
        <Card>
          <Card.Grid style={{width:'50%',display:'flex',flexDirection:'column',alignItems:'center',justifyContent:'space-around'}}>
            <Progress type="circle" percent={this.props.preprocessingPython} width={200} />
            <h4 style={{marginTop:30}}>Python</h4>
          </Card.Grid>
          <Card.Grid style={{width:'50%',display:'flex',flexDirection:'column',alignItems:'center',justifyContent:'space-around'}}>
            <Progress type="circle" percent={this.props.preprocessingJulia} width={200} />
            <h4 style={{marginTop:30}}>Julia</h4>
          </Card.Grid>
        </Card>
      </>
    )
  }

}

export default Preprocessing
