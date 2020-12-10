import React, { Component } from 'react';
import * as ROS3D from 'ros3d'

class Viewer extends Component {
  render() {
    return (
      <div id='viewer' style={{height:'100vh',width:'100vw',backgroundColor:'rgb(48,48,48)',padding:0}}/>
    );
  }

  /**
   * Setup all visualization elements when the page is loaded.
   */
  componentDidMount() {
    // Connect to ROS.
    // const ros = new ROSLIB.Ros({
    //   url: 'ws://localhost:9090'
    // });

    // Create the main viewer.
    const viewer = new ROS3D.Viewer({
      divID: 'viewer',
      width: 400,
      height: 300,
      antialias: true
    });

    // Setup a client to listen to TFs.
    // const tfClient = new ROSLIB.TFClient({
    //   ros: ros,
    //   angularThres: 0.01,
    //   transThres: 0.01,
    //   rate: 10.0,
    //   fixedFrame: '/my_frame'
    // });

    // Setup the marker client.
    // this.markerClient = new ROS3D.MarkerClient({
    //   ros: ros,
    //   tfClient: tfClient,
    //   topic: '/visualization_marker',
    //   rootObject: viewer.scene
    // });
  }
}

export default Viewer;
