import React, { Component } from 'react';
import ConfigCreator from './ConfigCreator';
import Detail from './config/Detail';

class Main extends Component {


  render() {
    return (
      <>
        <ConfigCreator meta={this.props.meta} config={this.props.config} onUpdate={this.props.onUpdate}/>
        {this.props.meta.selected ? <Detail meta={this.props.meta} config={this.props.config} onUpdate={this.props.onUpdate}/> : null}

      </>)
  }
}

export default Main;
