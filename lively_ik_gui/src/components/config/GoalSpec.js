import React from 'react';
import { List, Input, Tabs, Alert } from 'antd';

class GoalSpec extends React.Component {

  constructor(props) {
    super(props);
    this.state = {savedName:props.goalName,
                  showNameError:!this.validateName(props.name)};
  }

  validateName = (name) => {
    if (name === this.props.goalName) {
      return true
    } else if (this.props.goalNames.indexOf(name) >= 0) {
      return false
    } else if (name !== undefined && name.toLowerCase() === 'default') {
      return false
    } else if (name === '') {
      return false
    }
    return true
  }

  debounce = (func, timeout = 200) => {
    let timer;
    return (...args) => {
      clearTimeout(timer);
      timer = setTimeout(() => { func.apply(this, args); }, timeout);
    };
  }

  updateName = (event) => {
    let value = event.target.value;
    let showNameError = false;
    if (this.validateName(value)) {
      this.props.onUpdate({name:value})
    } else {
      showNameError = true
    }
    this.setState({savedName:value,showNameError:showNameError})
  }

  render() {
    return (
      <>
        <h3>Name</h3>
        <Input placeholder='Name this Goal'
               disabled={this.props.goalName === 'default'}
               value={this.props.goalName === 'default' ? 'Default' : this.state.savedName}
               onChange={(v)=>this.debounce(this.updateName(v))}/>
        {this.state.showNameError ? (
          <Alert
             message="Name Error"
             description="Please select a unique name for this mode."
             type="error"
             showIcon
           />
         ) : (<></>)
        }
      </>
    )
  }

}

export default GoalSpec
