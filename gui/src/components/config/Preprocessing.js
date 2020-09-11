import React from 'react';
import { Progress, Card, Button } from 'antd';

class Preprocessing extends React.Component {

  getStatus = (state) => {
    if (state.ok && state.progress < 100) {
      return 'active';
    } else if (state.ok && state.progress == 100) {
      return 'success';
    } else {
      return 'exception';
    }
  }

  render() {
    return (
      <>
        <h5 style={{backgroundColor:'#e8e8e8', borderRadius:3, padding:10}}>
          Complete the configuration by preprocessing.
        </h5>
        <Button style={{marginRight:5}} primary block onClick={this.props.beginPreprocess}>Begin Preprocess</Button>
        <Card>
          <Card.Grid style={{width:'25%',display:'flex',flexDirection:'column',alignItems:'center',justifyContent:'space-around'}}>
            <Progress type="circle" percent={this.props.preprocessingState.write_yaml.progress} status={this.getStatus(this.props.preprocessingState.write_yaml)} width={150} />
            <h4 style={{marginTop:30}}>YAML Config</h4>
          </Card.Grid>
          <Card.Grid style={{width:'25%',display:'flex',flexDirection:'column',alignItems:'center',justifyContent:'space-around'}}>
            <Progress type="circle" percent={this.props.preprocessingState.julia_nn.progress} status={this.getStatus(this.props.preprocessingState.julia_nn)} width={150} />
            <h4 style={{marginTop:30}}>Julia Neural Network</h4>
          </Card.Grid>
          <Card.Grid style={{width:'25%',display:'flex',flexDirection:'column',alignItems:'center',justifyContent:'space-around'}}>
            <Progress type="circle" percent={this.props.preprocessingState.julia_params.progress} status={this.getStatus(this.props.preprocessingState.julia_params)} width={150} />
            <h4 style={{marginTop:30}}>Julia Parameters</h4>
          </Card.Grid>
          <Card.Grid style={{width:'25%',display:'flex',flexDirection:'column',alignItems:'center',justifyContent:'space-around'}}>
            <Progress type="circle" percent={this.props.preprocessingState.python.progress} status={this.getStatus(this.props.preprocessingState.python)} width={150} />
            <h4 style={{marginTop:30}}>Python Neural Network</h4>
          </Card.Grid>
        </Card>
      </>
    )
  }

}

export default Preprocessing
