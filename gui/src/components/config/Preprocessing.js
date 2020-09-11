import React from 'react';
import { Progress, Card, Button, Popover } from 'antd';

const steps = ['write_yaml','julia_nn','julia_params','python'];

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

  getDescription = (step) => {
    switch (step) {
      case 'write_yaml':
        return 'Writing Yaml File';
      case 'julia_nn':
        return 'Training Julia Collison Network'
      case 'julia_params':
        return 'Configuring Julia Parameters'
      case 'python':
        return 'Training Python Collision Network'
    }
  }

  render() {
    console.log(this.props.preprocessingState);
    return (
      <>
        <h5 style={{backgroundColor:'#e8e8e8', borderRadius:3, padding:10}}>
          Complete the configuration by preprocessing.
        </h5>
        <Button style={{marginRight:5, marginBottom:20}} primary block onClick={this.props.beginPreprocess}>Begin Preprocess</Button>
        <Card style={{display:'flex',flexDirection:'row',justifyContent:'space-around'}}>
          {steps.map((step) => {
            return (
            <Popover content={this.getDescription(step)}
                     trigger='hover'
                     visible={0 < this.props.preprocessingState[step].progress && this.props.preprocessingState[step].progress < 100}>
              <Progress type='circle' style={{margin:5}} percent={this.props.preprocessingState[step].progress} status={this.getStatus(this.props.preprocessingState[step])} width={150} />
            </Popover>
          )})}
        </Card>
      </>
    )
  }

}

export default Preprocessing
