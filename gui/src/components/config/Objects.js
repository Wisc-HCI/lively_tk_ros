import React from 'react';
import { Button, Descriptions, Card, Select, InputNumber, Collapse, Empty, Input } from 'antd';
import {PlusOutlined,DeleteOutlined} from '@ant-design/icons';
const { Option } = Select;
const { Panel } = Collapse;

class Objects extends React.Component {

  createBox = (e) => {
    console.log(e)
    e.stopPropagation();
    let boxes = [...this.props.boxes];
    boxes.push({name:'Box '+(this.props.boxes.length+1).toString(),
                coordinate_frame: 0,
                parameters: [0.5,0.5,0.5],
                rotation: [0.0,0.0,0.0],
                translation: [0.0,0.0,0.0]});
    this.props.updateBoxes(boxes);
  }

  createSphere = (e) => {
    e.stopPropagation();
    let spheres = [...this.props.spheres];
    spheres.push({name:'Sphere '+(this.props.spheres.length+1).toString(),
                  coordinate_frame: 0,
                  parameters: 0.5,
                  rotation: [0.0,0.0,0.0],
                  translation: [0.0,0.0,0.0]});
    this.props.updateSpheres(spheres);
  }

  createEllipsoid = (e) => {
    e.stopPropagation();
    let ellipsoids = [...this.props.ellipsoids];
    ellipsoids.push({name:'Ell '+(this.props.ellipsoids.length+1).toString(),
                   coordinate_frame: 0,
                   parameters: [0.5,0.5,0.5],
                   rotation: [0.0,0.0,0.0],
                   translation: [0.0,0.0,0.0]});
    this.props.updateEllipsoids(ellipsoids);
  }

  createCapsule = (e) => {
    e.stopPropagation();
    let capsules = [...this.props.capsules];
    capsules.push({name:'Capsule '+(this.props.capsules.length+1).toString(),
                   coordinate_frame: 0,
                   parameters: [0.5,0.5],
                   rotation: [0.0,0.0,0.0],
                   translation: [0.0,0.0,0.0]});
    this.props.updateCapsules(capsules);
  }

  createCylinder = (e) => {
    e.stopPropagation();
    let cylinders = [...this.props.cylinders];
    cylinders.push({name:'Cylinder '+(this.props.cylinders.length+1).toString(),
                    coordinate_frame: 0,
                    parameters: [0.5,0.5],
                    rotation: [0.0,0.0,0.0],
                    translation: [0.0,0.0,0.0]});
    this.props.updateCylinders(cylinders);
  }

  removeBox = (idx) => {
    let boxes = [...this.props.boxes];
    boxes.splice(idx,1)
    this.props.updateBoxes(boxes);
  }

  removeSphere = (idx) => {
    let spheres = [...this.props.spheres];
    spheres.splice(idx,1)
    this.props.updateSpheres(spheres);
  }

  removeEllipsoid = (idx) => {
    let ellipsoids = [...this.props.ellipsoids];
    ellipsoids.splice(idx,1)
    this.props.updateEllipsoids(ellipsoids);
  }

  removeCapsule = (idx) => {
    let capsules = [...this.props.capsules];
    capsules.splice(idx,1)
    this.props.updateCapsules(capsules);
  }

  removeCylinder = (idx) => {
    let cylinders = [...this.props.cylinders];
    cylinders.splice(idx,1)
    this.props.updateCylinders(cylinders);
  }

  onChangeBoxFrame = (value,idx) => {
    let boxes = [...this.props.boxes];
    boxes[idx].coordinate_frame = value;
    this.props.updateBoxes(boxes);
  }

  onChangeBoxParameter = (value,idx,param) => {
    let boxes = [...this.props.boxes];
    boxes[idx].parameters[param] = value;
    this.props.updateBoxes(boxes);
  }

  onChangeBoxTranslation = (value,idx,dim) => {
    let boxes = [...this.props.boxes];
    boxes[idx].translation[dim] = value;
    this.props.updateBoxes(boxes);
  }

  onChangeBoxRotation = (value,idx,dim) => {
    let boxes = [...this.props.boxes];
    boxes[idx].rotation[dim] = value;
    this.props.updateBoxes(boxes);
  }

  onChangeSphereParameter = (value,idx) => {
    let spheres = [...this.props.spheres];
    spheres[idx].parameters = value;
    this.props.updateSpheres(spheres);
  }

  onChangeSphereTranslation = (value,idx,dim) => {
    let spheres = [...this.props.spheres];
    spheres[idx].translation[dim] = value;
    this.props.updateSpheres(spheres);
  }

  onChangeSphereRotation = (value,idx,dim) => {
    let spheres = [...this.props.spheres];
    spheres[idx].rotation[dim] = value;
    this.props.updateSpheres(spheres);
  }

  onChangeEllipsoidParameter = (value,idx,param) => {
    let ellipsoids = [...this.props.ellipsoids];
    ellipsoids[idx].parameters[param] = value;
    this.props.updateEllipsoids(ellipsoids);
  }

  onChangeEllipsoidTranslation = (value,idx,dim) => {
    let ellipsoids = [...this.props.ellipsoids];
    ellipsoids[idx].translation[dim] = value;
    this.props.updateEllipsoids(ellipsoids);
  }

  onChangeEllipsoidRotation = (value,idx,dim) => {
    let ellipsoids = [...this.props.ellipsoids];
    ellipsoids[idx].rotation[dim] = value;
    this.props.updateEllipsoids(ellipsoids);
  }

  onChangeCapsuleParameter = (value,idx,param) => {
    let capsules = [...this.props.capsules];
    capsules[idx].parameters[param] = value;
    this.props.updateCapsules(capsules);
  }

  onChangeCapsuleTranslation = (value,idx,dim) => {
    let capsules = [...this.props.capsules];
    capsules[idx].translation[dim] = value;
    this.props.updateCapsules(capsules);
  }

  onChangeCapsuleRotation = (value,idx,dim) => {
    let capsules = [...this.props.capsules];
    capsules[idx].rotation[dim] = value;
    this.props.updateCapsules(capsules);
  }

  onChangeCylinderParameter = (value,idx,param) => {
    let cylinders = [...this.props.cylinders];
    cylinders[idx].parameters[param] = value;
    this.props.updateCylinders(cylinders);
  }

  onChangeCylinderTranslation = (value,idx,dim) => {
    let cylinders = [...this.props.cylinders];
    cylinders[idx].translation[dim] = value;
    this.props.updateCylinders(cylinders);
  }

  onChangeCylinderRotation = (value,idx,dim) => {
    let cylinders = [...this.props.cylinders];
    cylinders[idx].rotation[dim] = value;
    this.props.updateCylinders(cylinders);
  }

  render() {
    return (
      <>
        <h5 style={{backgroundColor:'#e8e8e8', borderRadius:3, padding:10}}>
          Specify objects in the scene that can collide with the robot.
        </h5>
        <Collapse accordion>
          <Panel header="Boxes" key="1" extra={<Button onClick={(e)=>this.createBox(e)} icon={<PlusOutlined/>}/>}>
            <Card>
              {(this.props.boxes.length > 0) ? this.props.boxes.map((object,idx)=>{
                return (
                  <Card.Grid style={{width:'100%'}}>
                    <Descriptions column={2} extra={<Button danger icon={<DeleteOutlined onClick={(e)=>this.removeBox(idx)}/>}/>}>
                      <Descriptions.Item label='Name'>
                        <Input value={object.name} onChange={(e)=>this.onChangeBoxName(e,idx)} />
                      </Descriptions.Item>
                      <Descriptions.Item label='Frame'>
                        <Select style={{width:200}} onChange={(e)=>this.onChangeBoxFrame(e,idx)} value={object.coordinate_frame}>
                          <Option value={0}>Stationary</Option>
                          {this.props.jointOrdering.map((joint,jointIdx)=>{
                            return <Option value={jointIdx+1}>{joint}</Option>
                          })}
                        </Select>
                      </Descriptions.Item>
                      <Descriptions.Item label='Size'>
                        <InputNumber value={object.parameters[0]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeBoxParameter(e,idx,0)} />
                        <InputNumber value={object.parameters[1]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeBoxParameter(e,idx,1)} />
                        <InputNumber value={object.parameters[2]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeBoxParameter(e,idx,2)} />
                      </Descriptions.Item>
                      <Descriptions.Item label='Position'>
                        <InputNumber value={object.translation[0]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeBoxTranslation(e,idx,0)} />
                        <InputNumber value={object.translation[1]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeBoxTranslation(e,idx,1)} />
                        <InputNumber value={object.translation[2]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeBoxTranslation(e,idx,2)} />
                      </Descriptions.Item>
                      <Descriptions.Item label='Rotation'>
                        <InputNumber value={object.rotation[0]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeBoxRotation(e,idx,0)} />
                        <InputNumber value={object.rotation[1]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeBoxRotation(e,idx,1)} />
                        <InputNumber value={object.rotation[2]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeBoxRotation(e,idx,2)} />
                      </Descriptions.Item>
                    </Descriptions>
                  </Card.Grid>
                )
              }) : <Empty/>}
            </Card>
          </Panel>
          <Panel header="Spheres" key="2" extra={<Button onClick={(e)=>this.createSphere(e)} icon={<PlusOutlined/>}/>}>
            <Card>
              {(this.props.spheres.length > 0) ? this.props.spheres.map((object,idx)=>{
                return (
                  <Card.Grid style={{width:'100%'}}>
                    <Descriptions column={2} extra={<Button danger icon={<DeleteOutlined onClick={(e)=>this.removeSphere(idx)}/>}/>}>
                      <Descriptions.Item label='Name'>
                        <Input value={object.name} onChange={(e)=>this.onChangeSphereName(e,idx)} />
                      </Descriptions.Item>
                      <Descriptions.Item label='Frame'>
                        <Select style={{width:200}} onChange={(e)=>this.onChangeSphereFrame(e,idx)} value={object.coordinate_frame}>
                          <Option value={0}>Stationary</Option>
                          {this.props.jointOrdering.map((joint,jointIdx)=>{
                            return <Option value={jointIdx+1}>{joint}</Option>
                          })}
                        </Select>
                      </Descriptions.Item>
                      <Descriptions.Item label='Size'>
                        <InputNumber value={object.parameters[0]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeSphereParameter(e,idx)} />
                      </Descriptions.Item>
                      <Descriptions.Item label='Position'>
                        <InputNumber value={object.translation[0]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeSphereTranslation(e,idx,0)} />
                        <InputNumber value={object.translation[1]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeSphereTranslation(e,idx,1)} />
                        <InputNumber value={object.translation[2]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeSphereTranslation(e,idx,2)} />
                      </Descriptions.Item>
                      <Descriptions.Item label='Rotation'>
                        <InputNumber value={object.rotation[0]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeSphereRotation(e,idx,0)} />
                        <InputNumber value={object.rotation[1]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeSphereRotation(e,idx,1)} />
                        <InputNumber value={object.rotation[2]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeSphereRotation(e,idx,2)} />
                      </Descriptions.Item>
                    </Descriptions>
                  </Card.Grid>
                )
              }) : <Empty/>}
            </Card>
          </Panel>
          <Panel header="Ellipsoids" key="3" extra={<Button onClick={(e)=>this.createEllipsoid(e)} icon={<PlusOutlined/>}/>}>
            <Card>
              {(this.props.ellipsoids.length > 0) ? this.props.ellipsoids.map((object,idx)=>{
                return (
                  <Card.Grid style={{width:'100%'}}>
                    <Descriptions column={2} extra={<Button danger icon={<DeleteOutlined onClick={(e)=>this.removeEllipsoid(idx)}/>}/>}>
                      <Descriptions.Item label='Name'>
                        <Input value={object.name} onChange={(e)=>this.onChangeEllipsoidName(e,idx)} />
                      </Descriptions.Item>
                      <Descriptions.Item label='Frame'>
                        <Select style={{width:200}} onChange={(e)=>this.onChangeEllipsoidFrame(e,idx)} value={object.coordinate_frame}>
                          <Option value={0}>Stationary</Option>
                          {this.props.jointOrdering.map((joint,jointIdx)=>{
                            return <Option value={jointIdx+1}>{joint}</Option>
                          })}
                        </Select>
                      </Descriptions.Item>
                      <Descriptions.Item label='Size'>
                        <InputNumber value={object.parameters[0]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeEllipsoidParameter(e,idx,0)} />
                        <InputNumber value={object.parameters[1]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeEllipsoidParameter(e,idx,1)} />
                        <InputNumber value={object.parameters[2]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeEllipsoidParameter(e,idx,2)} />
                      </Descriptions.Item>
                      <Descriptions.Item label='Position'>
                        <InputNumber value={object.translation[0]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeEllipsoidTranslation(e,idx,0)} />
                        <InputNumber value={object.translation[1]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeEllipsoidTranslation(e,idx,1)} />
                        <InputNumber value={object.translation[2]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeEllipsoidTranslation(e,idx,2)} />
                      </Descriptions.Item>
                      <Descriptions.Item label='Rotation'>
                        <InputNumber value={object.rotation[0]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeEllipsoidRotation(e,idx,0)} />
                        <InputNumber value={object.rotation[1]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeEllipsoidRotation(e,idx,1)} />
                        <InputNumber value={object.rotation[2]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeEllipsoidRotation(e,idx,2)} />
                      </Descriptions.Item>
                    </Descriptions>
                  </Card.Grid>
                )
              }) : <Empty/>}
            </Card>
          </Panel>
          <Panel header="Capsules" key="4" extra={<Button onClick={(e)=>this.createCapsule(e)} icon={<PlusOutlined/>}/>}>
            <Card>
              {(this.props.capsules.length > 0) ? this.props.capsules.map((object,idx)=>{
                return (
                  <Card.Grid style={{width:'100%'}}>
                    <Descriptions column={2} extra={<Button danger icon={<DeleteOutlined onClick={(e)=>this.removeCapsule(idx)}/>}/>}>
                      <Descriptions.Item label='Name'>
                        <Input value={object.name} onChange={(e)=>this.onChangeCapsuleName(e,idx)} />
                      </Descriptions.Item>
                      <Descriptions.Item label='Frame'>
                        <Select style={{width:200}} onChange={(e)=>this.onChangeCapsuleFrame(e,idx)} value={object.coordinate_frame}>
                          <Option value={0}>Stationary</Option>
                          {this.props.jointOrdering.map((joint,jointIdx)=>{
                            return <Option value={jointIdx+1}>{joint}</Option>
                          })}
                        </Select>
                      </Descriptions.Item>
                      <Descriptions.Item label='Size'>
                        <InputNumber value={object.parameters[0]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeCapsuleParameter(e,idx,0)} />
                        <InputNumber value={object.parameters[1]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeCapsuleParameter(e,idx,1)} />
                      </Descriptions.Item>
                      <Descriptions.Item label='Position'>
                        <InputNumber value={object.translation[0]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeCapsuleTranslation(e,idx,0)} />
                        <InputNumber value={object.translation[1]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeCapsuleTranslation(e,idx,1)} />
                        <InputNumber value={object.translation[2]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeCapsuleTranslation(e,idx,2)} />
                      </Descriptions.Item>
                      <Descriptions.Item label='Rotation'>
                        <InputNumber value={object.rotation[0]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeCapsuleRotation(e,idx,0)} />
                        <InputNumber value={object.rotation[1]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeCapsuleRotation(e,idx,1)} />
                        <InputNumber value={object.rotation[2]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeCapsuleRotation(e,idx,2)} />
                      </Descriptions.Item>
                    </Descriptions>
                  </Card.Grid>
                )
              }) : <Empty/>}
            </Card>
          </Panel>
          <Panel header="Cylinders" key="5"extra={<Button onClick={(e)=>this.createCylinder(e)} icon={<PlusOutlined/>}/>}>
            <Card>
              {(this.props.cylinders.length > 0) ? this.props.cylinders.map((object,idx)=>{
                return (
                  <Card.Grid style={{width:'100%'}}>
                    <Descriptions column={2} extra={<Button danger icon={<DeleteOutlined onClick={(e)=>this.removeCylinder(idx)}/>}/>}>
                      <Descriptions.Item label='Name'>
                        <Input value={object.name} onChange={(e)=>this.onChangeCylinderName(e,idx)} />
                      </Descriptions.Item>
                      <Descriptions.Item label='Frame'>
                        <Select style={{width:200}} onChange={(e)=>this.onChangeCylinderFrame(e,idx)} value={object.coordinate_frame}>
                          <Option value={0}>Stationary</Option>
                          {this.props.jointOrdering.map((joint,jointIdx)=>{
                            return <Option value={jointIdx+1}>{joint}</Option>
                          })}
                        </Select>
                      </Descriptions.Item>
                      <Descriptions.Item label='Size'>
                        <InputNumber value={object.parameters[0]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeCylinderParameter(e,idx,0)} />
                        <InputNumber value={object.parameters[1]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeCylinderParameter(e,idx,1)} />
                      </Descriptions.Item>
                      <Descriptions.Item label='Position'>
                        <InputNumber value={object.translation[0]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeCylinderTranslation(e,idx,0)} />
                        <InputNumber value={object.translation[1]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeCylinderTranslation(e,idx,1)} />
                        <InputNumber value={object.translation[2]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeCylinderTranslation(e,idx,2)} />
                      </Descriptions.Item>
                      <Descriptions.Item label='Rotation'>
                        <InputNumber value={object.rotation[0]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeCylinderRotation(e,idx,0)} />
                        <InputNumber value={object.rotation[1]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeCylinderRotation(e,idx,1)} />
                        <InputNumber value={object.rotation[2]} min={0} max={25} step={0.05} onChange={(e)=>this.onChangeCylinderRotation(e,idx,2)} />
                      </Descriptions.Item>
                    </Descriptions>
                  </Card.Grid>
                )
              }) : <Empty/>}
            </Card>
          </Panel>
        </Collapse>
      </>
    )
  }

}

export default Objects
