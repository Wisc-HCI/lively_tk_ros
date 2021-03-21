import * as ROSLIB from 'roslib';
import * as ROS3D from 'ros3d';
import * as THREE from 'three-full';

class SimpleUrdf extends THREE.Object3D {
  constructor(options) {
    options = options || {};
    super(options);
    this.urdfModel = options.urdfModel;
    this.path = options.path || '/';
    this.tfClient = options.tfClient;
    this.tfPrefix = options.tfPrefix || '';
    this.loader = options.loader;

    this.__proto__ = THREE.Object3D.prototype;

    // load all models
    var links = this.urdfModel.links;
    for ( var l in links) {
      var link = links[l];
      for( var i=0; i<link.visuals.length; i++ ) {
        var visual = link.visuals[i];
        if (visual && visual.geometry) {
          // Save frameID
          var frameID = this.tfPrefix + '/' + link.name;
          // Save color material
          var colorMaterial = null;
          if (visual.material && visual.material.color) {
            var color = visual.material && visual.material.color;
            colorMaterial = ROS3D.makeColorMaterial(color.r, color.g, color.b, color.a);
          }
          if (visual.geometry.type === ROSLIB.URDF_MESH) {
            var uri = visual.geometry.filename;
            // strips package://
            var tmpIndex = uri.indexOf('package://');
            if (tmpIndex !== -1) {
              uri = uri.substr(tmpIndex + ('package://').length);
            }
            var fileType = uri.substr(-3).toLowerCase();

            if (ROS3D.MeshLoader.loaders[fileType]) {
              // create the model
              var mesh = new ROS3D.MeshResource({
                path : this.path,
                resource : uri,
                loader : this.loader,
                material : colorMaterial
              });

              // check for a scale
              if(link.visuals[i].geometry.scale) {
                mesh.scale.copy(visual.geometry.scale);
              }

              // create a scene node with the model
              var sceneNode = new ROS3D.SceneNode({
                frameID : frameID,
                  pose : visual.origin,
                  tfClient : this.tfClient,
                  object : mesh
              });
              sceneNode.name = visual.name
              this.add(sceneNode);
            } else {
              console.warn('Could not load geometry mesh: '+uri);
            }
          } else {
            var shapeMesh = this.createShapeMesh(visual, options);
            // Create a scene node with the shape
            var scene = new ROS3D.SceneNode({
              frameID: frameID,
                pose: visual.origin,
                tfClient: this.tfClient,
                object: shapeMesh
            });
            scene.name = visual.name
            this.add(scene);
          }
        }
      }
    }

  }

  createShapeMesh = (visual,options) => {
    var colorMaterial = null;
    if (!colorMaterial) {
      colorMaterial = ROS3D.makeColorMaterial(0, 0, 0, 1);
    }
    var shapeMesh;
    // Create a shape
    switch (visual.geometry.type) {
      case ROSLIB.URDF_BOX:
        var dimension = visual.geometry.dimension;
        var cube = new THREE.BoxGeometry(dimension.x, dimension.y, dimension.z);
        shapeMesh = new THREE.Mesh(cube, colorMaterial);
        break;
      case ROSLIB.URDF_CYLINDER:
        var radius = visual.geometry.radius;
        var length = visual.geometry.length;
        var cylinder = new THREE.CylinderGeometry(radius, radius, length, 16, 1, false);
        shapeMesh = new THREE.Mesh(cylinder, colorMaterial);
        shapeMesh.quaternion.setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI * 0.5);
        break;
      case ROSLIB.URDF_SPHERE:
        var sphere = new THREE.SphereGeometry(visual.geometry.radius, 16);
        shapeMesh = new THREE.Mesh(sphere, colorMaterial);
        break;
    }

    return shapeMesh;
  }

  unsubscribeTf = () => {
    this.children.forEach(function(n) {
      if (typeof n.unsubscribeTf === 'function') { n.unsubscribeTf(); }
    });
  }
}

export default SimpleUrdf;
