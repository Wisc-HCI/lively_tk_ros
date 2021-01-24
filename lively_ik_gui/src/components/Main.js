import React, { Component } from 'react';
import ConfigCreator from './ConfigCreator';
import Commander from './Commander';
import { Tabs } from 'antd';
const { TabPane } = Tabs;

class Main extends Component {

  state = {maxStep:0,step:0}

  render() {
    return (
      <Tabs defaultActiveKey="1" centered style={{backgroundColor:'white',height:'100%'}}>
        <TabPane tab="Setup" key="1">
          <ConfigCreator maxStep={this.state.maxStep} step={this.state.step} config={this.props.config}/>
        </TabPane>
        <TabPane enabled={this.state.maxStep === 8 } tab="Run" key="2">
          <Commander/>
        </TabPane>
      </Tabs>)
  }
}

export default Main;
