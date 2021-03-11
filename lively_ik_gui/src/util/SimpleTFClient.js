import * as ROSLIB from 'roslib';

/**
 * A TF Client that listens to TFs from tf2_web_republisher.
 *
 *  @constructor
 *  @param ros - the ROSLIB.Ros connection handle
 */
class SimpleTFClient {

  constructor(ros) {
    this.ros = ros;
    this.tf_topic = new ROSLIB.Topic({
      ros: this.ros,
      name: '/lively_ik/tf_updates',
      messageType: 'tf2_web_republisher_msgs/TFArray'
    });
    this.request_topic = new ROSLIB.Topic({
      ros: this.ros,
      name: '/lively_ik/tf_request',
      messageType: 'std_msgs/String'
    });
    this.fixedFrame = 'world';
    this.frameInfos = {};
    this.tf_topic.subscribe((tf) => {
      tf.transforms.forEach((transform)=>{
        var frameID = transform.child_frame_id;
        if (frameID[0] === '/')
        {
          frameID = frameID.substring(1);
        }
        var info = this.frameInfos['/'+frameID];
        if (info) {
          info.transform = new ROSLIB.Transform({
            translation : transform.transform.translation,
            rotation : transform.transform.rotation
          });
          info.cbs.forEach(function(cb) {
            cb(info.transform);
          });
        }
      })
    })
  }

  unsubscribe = (frameID, callback) => {
    var info = this.frameInfos[frameID];
    for (var cbs = info && (info.cbs || []), idx = cbs.length; idx--;) {
      if (cbs[idx] === callback) {
        cbs.splice(idx, 1);
      }
    }
    if (!callback || cbs.length === 0) {
      delete this.frameInfos[frameID];
    }
  }

  subscribe = (frameID,callback) => {
    let info = this.frameInfos[frameID];
    if (info) {
      this.frameInfos[frameID].cbs.push(callback)
    } else {
      this.frameInfos[frameID] = {transform:ROSLIB.Transform(),cbs:[callback]}
    }
    this.frameInfos[frameID].cbs.forEach((cb)=>cb(this.frameInfos[frameID].transform));
    this.request_topic.publish({data:JSON.stringify({source_frame:frameID,target_frame:this.fixedFrame})})
  }

    dispose = () => {
      this.tf_topic.unsubscribe();
    };
}

export default SimpleTFClient;
