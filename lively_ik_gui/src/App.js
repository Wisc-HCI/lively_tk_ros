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
import { message, Layout, Empty, Card, Space } from 'antd';
import * as ROSLIB from 'roslib';
import SplitPane from 'react-split-pane';
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
                  config:{
                    axis_types:[],
                    ee_fixed_joints:[],
                    base_link_motion_bounds:[[0,0],[0,0],[0,0]],
                    static_environment:{
                      cuboids:[],
                      spheres:[],
                      pcs:[]
                    },
                    fixed_frame:'',
                    goals:[],
                    joint_limits:[],
                    joint_names:[],
                    joint_ordering:[],
                    joint_types:[],
                    mode_control:'absolute',
                    mode_environment:'ECAA',
                    nn_jointpoint:[],
                    nn_main:[],
                    objectives:[],
                    states:[],
                    robot_link_radius:0.05,
                    rot_offsets:[],
                    starting_config:[],
                    urdf:'<?xml version="1.0" ?><robot name="default" xmlns:xacro="http://www.ros.org/wiki/xacro"><link name="base_link"/><joint name="default_joint" type="fixed"><parent link="base_link" /><child link="default_link" /><origin xyz="0 0 0" rpy="0 0 0" /></joint><link name="default_link"/></robot>',
                    velocity_limits:[],
                    disp_offsets:[],
                    displacements:[]
                  },
                  meta:{
                    valid_urdf: false,
                    valid_robot: false,
                    valid_nn: false,
                    valid_config: false,
                    valid_solver: false,
                    displayed_state:[],
                    links: [],
                    dynamic_joints: [],
                    fixed_joints: []
                  }
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
      this.setState({connected:true,showUploader:true});

      // Create Subscriber and Publisher
      this.serverUpdate = new ROSLIB.Topic({
        ros: this.ros,
        name: '/lively_ik/server_updates',
        messageType: 'std_msgs/String'
      });

      this.serverUpdate.subscribe(message => {
        this.updateFromServer(message.data);
      });

      this.guiUpdate = new ROSLIB.Topic({
        ros: this.ros,
        name: '/lively_ik/gui_updates',
        messageType: 'std_msgs/String'
      })

    })
    this.ros.on('close', ()=>{
      // We successfully closed our connection to the ROS socket.
      console.log("Connection Closed")
      this.setState({connected:false})
    })

    // Setup the subscriptions/etc.
    // TODO


    this.setState(login);
  }

  updateFromServer(data) {
    let update = JSON.parse(data);
    if (update.directive === 'update') {
      this.setState({config:update.config,meta:update.meta});
    }
  }

  updateToServer(data) {
    this.guiUpdate.publish({data:JSON.stringify(data)})
  }

  handleUpload = (config) => {
    console.log(config);
    this.setState({config:config,showUploader:false})
    this.updateToServer({directive:'update',config:config})
    // TODO: update the viewer's base link
  }

  handleUploadCancel = () => {
    this.setState({showUploader:false})
  }

  connect = (prefix,host) => {
    this.setState({prefix:prefix,host:host});
    this.ros.connect(prefix+'://'+host);
  }

  getContent = () => {
    if (this.state.connected) {
      return (
        <SplitPane split='vertical' defaultSize="50%" style={{width: '100%', display:'flex', height:'calc(100vh - 48pt)'}}>
          <Scene ros={this.ros} baseLink={this.state.config.fixed_frame} urdf={this.state.config.urdf}/>
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
        <Header style={{ position: 'fixed', zIndex: 1, width: '100%', backgroundColor:'black' }}>
          <Space align="center">
            <SettingFilled style={{fontSize:'25pt',textAlign:'center', color:'white',marginRight:5}}/>
            <span style={{fontSize:'25pt',color:'white'}}>LivelyIK</span>
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
