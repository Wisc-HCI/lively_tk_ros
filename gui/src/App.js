import React from 'react';
import {MenuUnfoldOutlined,
        MenuFoldOutlined,
        SettingFilled,
        PlusOutlined
       } from '@ant-design/icons';
import './App.css';
import 'antd/dist/antd.css';
import io from 'socket.io-client';
import Connection from './components/Connection';
import ConfigCreator from './components/ConfigCreator';
import { message, Layout, Menu, Empty, Card } from 'antd';
const { Header, Sider, Content, Footer } = Layout;

const Applications = {
  config_creator:{logo:PlusOutlined,component:ConfigCreator}
}

class App extends React.Component {

  constructor(props) {
    super(props);
    this.state = {host:'localhost:5000',
                  prefix:'http',
                  connected:false,
                  activeApp:null,
                  collapsed: false,
                  appList:[]
                 };
  }

  toggle = () => {
    this.setState({
      collapsed: !this.state.collapsed,
    });
  };

  componentDidMount() {
    let login = JSON.parse(localStorage.getItem('login')) || {host:'localhost:5000',prefix:'http'};
    this.setState(login);
  }

  connect = (prefix,host) => {
    this.setState({prefix:prefix,host:host});
    this.socket = io(prefix+'://'+host);
    this.socket.on('connect_error',()=>{
        message.error("Could not connect!");
        this.socket.disconnect();
        this.setState({connected:false,activeApp:null})
      })
    this.socket.on('connect',()=>{
      console.log("Connection!")
      message.info("Connected!");
      localStorage.setItem('login',JSON.stringify({host:host,prefix:prefix}));
      this.setState({connected:true});
    })
    this.socket.on('get_app_list_response',(data)=>{
      console.log('App Updates!',data)
      this.setState({appList:data.apps});
    })
    this.socket.on('select_app_response',(data)=>{
      console.log('App Selected!',data)
      if (data.success) {
        this.setState({activeApp:data.app})
      } else {
        message.error("Could not initialize app!");
      }
    })
  }

  setApplication = (code) => {
    this.socket.emit('set_active_app',{app:code})
  }

  getApplication = () => {
    let Module = Applications[this.state.activeApp].component;
    return <Module socket={this.socket}/>
  }

  getContent = () => {
    if (this.state.connected && this.state.activeApp !== null) {
      return this.getApplication()
    } else if (this.state.connected) {
      return (
        <Card style={{margin:10}}>
          <Empty description="Select an app on the left to begin." />
        </Card>
      )
    } else {
      return (
        <Connection host={this.state.host} prefix={this.state.prefix} connect={(prefix,host)=>this.connect(prefix,host)}/>
      )
    }
  }

  getAppButtons = () => {
    let items = [];
    if (this.state.connected) {

      this.state.appList.forEach((item)=>{
        let SidebarLogo = Applications[item.code].logo;
        items.push(<Menu.Item style={{flex:1,alignContent:'center'}} key={item.code} icon={<SidebarLogo/>} onClick={()=>this.setApplication(item.code)}>
                      {item.name}
                   </Menu.Item>)
      })
      return items
    }
    return items
  }


  render() {
    return (
      <Layout className="layout">
        <Sider trigger={null} collapsible collapsed={this.state.collapsed} style={{backgroundColor:'black'}}>
          <span className='logo' style={{display:'flex',flexDirection:'row',flex:1,alignItems:'center',justifyContent:'space-around'}}><SettingFilled style={{textAlign:'center', marginRight:this.state.collapsed ? 0 : 5}}/>{!this.state.collapsed ? <span>Lively Apps</span> : <></>}</span>
          <Menu theme="dark" mode="inline" defaultSelectedKeys={['1']} style={{backgroundColor:'black'}}>
            {this.getAppButtons()}
          </Menu>
        </Sider>
        <Layout className='site-layout'>
          <Header className="site-layout-background" style={{ padding: 0 }}>
            {React.createElement(this.state.collapsed ? MenuUnfoldOutlined : MenuFoldOutlined, {
              className: 'trigger-highlight-on-hover',
              onClick: this.toggle,
            })}
          </Header>
          <Content style={{ minHeight: 'calc(100vh - 110px)', backgroundColor: "#46484d" }}>
            {this.getContent()}
          </Content>
          <Footer style={{ height: 50, textAlign: 'center', backgroundColor: "#46484d"  }}>LivelyIK ©2020 Created by Andrew Schoen</Footer>
        </Layout>

      </Layout>
    );
  }

}

export default App;
