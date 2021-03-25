import React, { Component } from 'react';
import ConfigCreator from './ConfigCreator';
import JointSpec from './config/JointSpec';
import ModeSpec from './config/ModeSpec';
import GoalSpec from './config/GoalSpec';
import ObjectiveSpec from './config/ObjectiveSpec';

export default function Main(props) {

  return (
      <>
        <ConfigCreator meta={props.meta} config={props.config} onUpdate={props.onUpdate}/>
        <JointSpec meta={props.meta} config={props.config} onUpdate={props.onUpdate}/>
        <GoalSpec meta={props.meta} config={props.config} onUpdate={props.onUpdate}/>
        <ModeSpec meta={props.meta} config={props.config} onUpdate={props.onUpdate}/>
        <ObjectiveSpec meta={props.meta} config={props.config} onUpdate={props.onUpdate}/>
      </>)
}

/*
{props.meta.selected && (props.meta.selected.type === 'objective') && <ObjectiveSpec meta={props.meta} config={props.config} onUpdate={props.onUpdate}/>}
{props.meta.selected && (props.meta.selected.type === 'goal') && }
{props.meta.selected && (props.meta.selected.type === 'mode') && }
*/
