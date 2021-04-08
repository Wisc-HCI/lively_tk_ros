import React from 'react';
import { Input, Select, Card } from 'antd';
const { Option } = Select;

class Connection extends React.Component {

  constructor(props) {
    super(props);
    this.state = {host:props.host,prefix:props.prefix};
  }

  connect = () => {
    this.props.connect(this.state.prefix,this.state.host);
  }

  onChangeHost = (event) => {
    this.setState({host:event.target.value});
  }

  onChangePrefix = (event) => {
    this.setState({prefix:event});
  }

  render() {
    const selectBefore = (
      <Select defaultValue={this.state.prefix} onChange={this.onChangePrefix}>
        <Option value="ws">ws://</Option>
        <Option value="wss">wss://</Option>
      </Select>
    )
    return (
      <Card title="Connect" style={{margin:10}}>
        <Input.Search addonBefore={selectBefore}
                      defaultValue={this.state.host}
                      enterButton="Connect"
                      value={this.state.host}
                      onSearch={this.connect}
                      onChange={this.onChangeHost}/>
      </Card>
    )
  }

}

export default Connection
