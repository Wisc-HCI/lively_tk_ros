import React, { Component } from 'react';
import * as ROS3D from 'ros3d';
import * as ROSLIB from 'roslib';
import * as THREE from 'three-full';

// import sizeMe from 'react-sizeme';
import { withResizeDetector } from 'react-resize-detector';

class Scene extends Component {

  render() {
    return (
      <div style={{backgroundColor:'#303030',color:'white',height:'100%',width:'100%'}} id="scene"/>
    );
  }


   componentDidUpdate(prevProps) {
    const { width, height, fixedFrame, urdf } = this.props;

    if (width !== prevProps.width || height !== prevProps.height) {
      this.viewer.resize(width,height)
    }
    if (fixedFrame !== prevProps.fixedFrame || urdf !== prevProps.urdf) {
      this.setupViewer(this.props.urdf,this.props.fixedFrame)
    }

  }

  setupViewer(urdf,fixedFrame) {
    this.viewer = null;
    document.getElementById('scene').innerHTML = "";
    // Create the main viewer.
    this.viewer = new ROS3D.Viewer({
      divID: 'scene',
      width: this.props.width,
      height: this.props.height,
      antialias: true,
      background: '#303030',
      intensity: .65
    });
    // this.handleResize();
    this.viewer.addObject(new ROS3D.Grid({
           color: '#0181c4',
           cellSize: 0.5,
           num_cells: 20,
           lineWidth: 2
    }));
    // Setup a client to listen to TFs.
    this.tfClient = new ROSLIB.TFClient({
      ros: this.props.ros,
      angularThres: 0.01,
      transThres: 0.01,
      rate: 10.0,
      fixedFrame: 'world'
    });
    // Setup the marker client.
    this.markerClient = new ROS3D.MarkerClient({
      ros: this.props.ros,
      tfClient: this.tfClient,
      topic: '/visualization_marker',
      rootObject: this.viewer.scene
    });
    this.robotModel = new ROSLIB.UrdfModel({
      string:this.props.urdf.replace(/package:\/\//g, process.env.PUBLIC_URL + 'assets/')
    });
    console.log(this.robotModel);
    this.robot = new ROS3D.Urdf({
      urdfModel: this.robotModel,
      path: '/',
      tfClient: this.tfClient,
      tfPrefix: '',
      loader: THREE.ColladaLoader
    })
    console.log(this.robot);
    this.viewer.scene.add(this.robot);
  }

   componentDidMount() {
     this.setupViewer(this.props.urdf,this.props.fixedFrame)
  }
}

export default withResizeDetector(Scene);
