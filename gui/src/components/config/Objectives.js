import React from 'react';
import { List, Button, Form } from 'antd';

class Objectives extends React.Component {

  constructor(props) {
    super(props);
    this.state = {editingObjective:-1};
  }

  updateObjectiveAtIdx = (objective,idx) => {
    let objectives = [...this.props.objectives];
    objectives[idx] = objective;
    this.props.updateObjectives(objectives);
  }

  addObjective = () => {

  }

  editObjective = (idx) => {
    this.setState({editingObjective:idx})
  }

  removeObjective = (idx) => {

  }

  getObjective = (objective,idx) => {
    let active = idx === this.state.editingObjective;
    return (
      <Form>
        <Form.Item label="Type">
        </Form.Item>
        <Form.Item label="Index">
        </Form.Item>
      </Form>
    );
  }

  render() {
    return (
      <>
        <h5 style={{backgroundColor:'#e8e8e8', borderRadius:3, padding:10}}>
          Specify objectives to use with LivelyIK
        </h5>
        <List header={<div>Objectives</div>} bordered>
          {this.props.objectives.map(this.getObjective)}
        </List>
      </>
    )
  }

}

export default Objectives
