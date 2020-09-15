import React from 'react';
import { message, Card } from 'antd';
import YAML from 'yaml'

const toCamel = (s) => {
  return s.replace(/([-_][a-z])/ig, ($1) => {
    return $1.toUpperCase()
      .replace('-', '')
      .replace('_', '');
  });
};

class Commander extends React.Component {

  render() {
    return (
    <>
      <Card title="Commander" size='small' style={{margin:10}}>
        Coming Soon
      </Card>
    </>
    )
  }

}

export default Commander
