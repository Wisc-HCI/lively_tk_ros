import React, { Component } from 'react';
import ConfigCreator from './ConfigCreator';
import Commander from './Commander';
import { Tabs } from 'antd';
const { TabPane } = Tabs;

class Main extends Component {

  render() {
    return (
      <Tabs defaultActiveKey="1" centered style={{backgroundColor:'white',height:'100%'}}>
        <TabPane tab="Setup" key="1">
          <ConfigCreator meta={this.props.meta} config={this.props.config} onUpdate={this.props.onUpdate}/>
        </TabPane>
        <TabPane enabled={this.props.meta.valid_solver === true} tab="Run" key="2">
          <Commander/>
        </TabPane>
      </Tabs>)
  }
}

export default Main;
