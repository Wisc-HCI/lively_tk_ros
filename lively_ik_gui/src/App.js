import React from 'react';
import {SettingFilled} from '@ant-design/icons';
import './App.css';
import 'antd/dist/antd.css';
import Connection from './components/Connection';
// import ConfigCreator from './components/ConfigCreator';
// import Commander from './components/Commander';
import Scene from './components/Scene';
import Uploader from './components/Uploader';
import Main from './components/Main';
import { message, Layout, Space, Spin, Button } from 'antd';
import * as ROSLIB from 'roslib';
import SplitPane from 'react-split-pane';
import { defaultConfig, defaultMeta } from './util/Default';
// import {Resizable} from 're-resizable';
const { Header, Sider, Content } = Layout;

class App extends React.Component {

  constructor(props) {
    super(props);
    this.state = {host:'localhost:9090',
                  prefix:'ws',
                  connected:false,
                  sidebarCollapsed: true,
                  showUploader:false,
                  config:JSON.parse(JSON.stringify(defaultConfig)),
                  meta:JSON.parse(JSON.stringify(defaultMeta))
                 };
  }

  componentDidMount() {
    let login = JSON.parse(localStorage.getItem('login')) || {host:'localhost:9090',prefix:'ws'};
    this.ros = new ROSLIB.Ros();
    this.ros.on('error', ()=>{
      // There was an error connecting to the ROS socket.
      message.error("Could not connect!");
      this.setState({connected:false})
    })
    this.ros.on('connection', ()=>{
      // We successfully connected to the ROS socket.
      console.log("Connection!")
      message.info("Connected!");
      localStorage.setItem('login',JSON.stringify({host:login.host,prefix:login.prefix}));

      // Create Subscriber and Publisher
      this.serverUpdate = new ROSLIB.Topic({
        ros: this.ros,
        name: '/lively_ik/server_updates',
        messageType: 'std_msgs/String'
      });

      this.serverUpdate.subscribe(message => {
        this.updateFromServer(message.data);
      });

      // For debugging marker msgs
      this.markerUpdate = new ROSLIB.Topic({
        ros: this.ros,
        name: '/visualization_marker',
        messageType: 'visualization_msgs/Marker'
      });

      this.markerUpdate.subscribe(message => {
        console.log('MARKER UPDATE:');
        console.log(message);
      });


      this.guiUpdate = new ROSLIB.Topic({
        ros: this.ros,
        name: '/lively_ik/gui_updates',
        messageType: 'std_msgs/String'
      })
      this.updateToServer({directive:'connect'})

    })
    this.ros.on('close', ()=>{
      // We successfully closed our connection to the ROS socket.
      console.log("Connection Closed")
      this.setState({connected:false})
    })


    this.setState(login);
  }

  updateFromServer(data) {
    let update = JSON.parse(data);
    console.log(update);
    if (update.directive === 'update') {
      this.setState((currentState)=>{
        let {config, meta} = update;
        let newState = {...currentState};
        if (config !== undefined) {
          Object.keys(config).forEach(field=>{
            newState.config[field] = config[field]
          })
        }
        if (meta !== undefined) {
          Object.keys(meta).forEach(field=>{
            newState.meta[field] = meta[field]
          })
        }
        newState.connected = true;
        return newState
      });
    }
  }

  updateToServer(data) {
    this.guiUpdate.publish({data:JSON.stringify(data)})
  }

  handleUpload = (config) => {
    this.setState({config:config,showUploader:false})
    this.updateToServer({directive:'update',config:config})
  }

  handleUploadCancel = () => {
    this.updateToServer({directive:'clear'})
    this.setState({showUploader:false})
  }

  handleUploadContinue = () => {
    this.updateToServer({directive:'resume'})
    this.setState({showUploader:false})
  }

  connect = (prefix,host) => {
    this.setState({prefix:prefix,host:host});
    this.ros.connect(prefix+'://'+host);
  }

  getContent = () => {
    if (this.state.connected) {
      return (
        <SplitPane split='vertical' defaultSize="50%" style={{width: '100%', display:'flex', height:'calc(100vh - 48pt)', backgroundColor:'white'}}>
          <Scene ros={this.ros} baseLink={this.state.config.fixed_frame} urdf={this.state.config.urdf} connected={this.state.connected}/>
          <Main meta={this.state.meta} config={this.state.config} onUpdate={(data)=>this.updateToServer(data)}/>
        </SplitPane>
      )
    } else {
      return (
        <Connection host={this.state.host} prefix={this.state.prefix} connect={(prefix,host)=>this.connect(prefix,host)}/>
      )
    }
  }
  
  render() {
    return (
      <>
      <Layout>
        <Header style={{ position: 'fixed', zIndex: 1, width: '100%', backgroundColor:'black', display:'flex', justifyContent:'space-between' }}>
          <Space align="baseline">
            <SettingFilled style={{fontSize:'25pt',textAlign:'center', color:'white',marginRight:5}}/>
            <span style={{fontSize:'25pt',color:'white'}}>LivelyIK</span>
            <div style={{display:'flex'}}/>
          </Space>
          <Space align="center">
            {this.state.meta.updating ? <Spin/> : <span>Up To Date</span>}
            {this.state.connected ? <Button ghost onClick={()=>this.setState({showUploader:true})} style={{marginLeft:10}}>Upload</Button> : <></>}
            {this.state.meta.valid_config ? <Button ghost href={`data:text/json;charset=utf-8,${encodeURIComponent(JSON.stringify(this.state.config))}`} download="config.json" style={{marginLeft:10}}>Export</Button> : <></>}
          </Space>
        </Header>
        <Content style={{ height: 'calc(100vh - 48pt)', backgroundColor: "#46484d", marginTop:'48pt', padding:0}}>
          <div style={{display:'flex',alignItems:'center',justifyContent:'center',height:'calc(100vh - 48pt)'}}>
            {this.getContent()}
          </div>
        </Content>
        <Sider style={{backgroundColor:"#81848C"}} collapsed={this.state.sidebarCollapsed} width="30%" collapsedWidth={0}>Sider</Sider>
      </Layout>
      <Uploader visible={this.state.showUploader} onUpload={this.handleUpload} onCancel={this.handleUploadCancel}/>
      </>
    );
  }

}

export default App;
