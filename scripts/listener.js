#!/usr/bin/env node

/************************************************************************
 Copyright (c) 2018, DRONES Lab, University at Buffalo
 Copyright (c) 2018, Seyed Mahdi Shamsi

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
************************************************************************/

'use strict';
/**
 * This example demonstrates simple receiving of messages over the ROS system.
 */

// Require rosnodejs itself
const rosnodejs = require('rosnodejs');
// Requires the std_msgs message package
const std_msgs = rosnodejs.require('std_msgs').msg;

const path = require('path');
const fs = require('fs');
const util = require('util');
const shell = require('shelljs');

//
const webppl = require('webppl');

const TOPICS_REFRESH_DELAY_ms = 100;


var globalStore = {}
globalStore.packages = {
  robot: require('./robot.js')
}

function getSubValue(object, trace) {
  let value = object;
  let keys = trace.split('/')

  for(let i in keys){
    let keyName = keys[i]
    
    if(keyName=='')
      continue

    value = value[keyName]
  }

  return value
}

function extend(target, object){

  for(const key in object){
    if(target[key]===undefined || !(object[key] instanceof Object))
      target[key] = object[key]
    else 
      extend(target[key], object[key])
  }
} 

function setSubValue(object, trace, data) {
  let value = object;
  let keys = trace.split('/')

  for(let i in keys){
    let keyName = keys[i]
    
    if(keyName=='')
      continue

    if(i == keys.length-1){
      extend(value[keyName], data)
      return object
    }

    value = value[keyName]
  }

  extend(object, data)
  return object
} 

function resolveValueName(valueName, callback){
  shell.exec('rostopic list ', {silent:true}, (code, stdout, stderr)=>{
    if(code){
      callback(null)
      return
    }

    const topics = stdout.split('\n')

    for(let topicIndex in topics){
      let topicName = topics[topicIndex];
      if(topicName!='' && valueName.startsWith(topicName)){
        callback({
          topic: topicName,
          key: valueName.substring(topicName.length)
        })
        return
      }
    }

    callback(null)
  })
}

function getTopicType(topicName){
  
  var res = shell.exec('rostopic type ' + topicName, {silent:true})

  if(res.code)
    return null

  res = res.replace(/\r?\n|\r/g, "");
  res = res.split('/')
  return {
    pkg: res[0],
    msg: res[1]
  }
}


function evaluate(object, globalStore) {
  var output = {}
  object(globalStore, function(s, k, a){
    output = {
      s: s,
      k: k,
      a: a
    }
  }, '')

  return output
}

function subscribeValue(nodeHandle, value, callback){
  let topicName = value.topic;
  let topicType = getTopicType(topicName);          

  if(!topicType)
    return

  const topic_msgs = rosnodejs.require(topicType.pkg).msg;
  let topicMsgType = topic_msgs[topicType.msg];

  let sub = nodeHandle.subscribe(topicName, topicMsgType,
    (data) => { callback(getSubValue(data, value.key)) });
}

function advertiseValue(nodeHandle, value){
  let topicName = value.topic;
  let topicType = getTopicType(topicName);          

  if(!topicType)
    return

  const topic_msgs = rosnodejs.require(topicType.pkg).msg;
  let topicMsgType = topic_msgs[topicType.msg];

  let pub = nodeHandle.advertise(topicName, topicMsgType);

  return function(data) {
    var msg = new topicMsgType()

    msg = setSubValue(msg, value.key, data)

    // console.log(topicName)
    // console.log(msg)
    pub.publish(msg)
  }
}

function registerValue(nodeHandle, valueName, callback){
  let attempt = function() {
    console.log('trying ' + valueName)
    resolveValueName(valueName, (value)=>{
      if(value){
        subscribeValue(nodeHandle, value, callback);
        console.log(valueName + " connected!");
      } else {
        setTimeout(attempt, TOPICS_REFRESH_DELAY_ms)
      }

    });
  };

  attempt();
}

function tryAdvertiseValue(nodeHandle, valueName, callback){
  let attempt = function() {
    // console.log('trying ' + valueName)
    resolveValueName(valueName, (value)=>{
      if(value){
        let publish = advertiseValue(nodeHandle, value);
        callback(publish)
        console.log(valueName + " connected!");
      } else {
        setTimeout(attempt, TOPICS_REFRESH_DELAY_ms)
      }

    });
  };

  attempt();
}

function runningInBrowser() {
  return (typeof window !== 'undefined');
}

var trampolineRunners = {
  web: function(yieldEvery) {
    yieldEvery = yieldEvery || 100;
    var f = function(t, wrappedF) {
      var lastPauseTime = Date.now();

      if (f.__cancel__) {
        f.__cancel__ = false;
      } else {
        while (t) {
          var currTime = Date.now();
          if (currTime - lastPauseTime > yieldEvery) {
            // NB: return is crucial here as it exits the while loop
            // and i'm using return rather than break because we might
            // one day want to cancel the timer
            return setTimeout(function() { wrappedF(t, wrappedF); }, 0);
          } else {
            t = t();
          }
        }
      }
    };
    return f;
  },
  cli: function() {
    return function(t) {
      while (t) {
        t = t()
      }
    };
  }
}

function wrapWithHandler(f, handler) {
  return function(x, y) {
    try {
      return f(x, y);
    } catch (e) {
      handler(e);
    }
  };
}

function wrapRunner(baseRunner, handlers) {
  var wrappedRunner = handlers.reduce(wrapWithHandler, baseRunner);
  var runner = function(t) { return wrappedRunner(t, runner); };
  return runner;
}

function getCompiledObject(code, runnerName) {

  if (runnerName === undefined) {
    runnerName = runningInBrowser() ? 'web' : 'cli'
  }

  // On error, throw out the stack. We don't support recovering the
  // stack from here.
  var handler = function(error) {
    throw 'webpplEval error:\n' + error;
  };
  var baseRunner = trampolineRunners[runnerName]();
  var runner = wrapRunner(baseRunner, [handler]);

  var compiledCode = webppl.compile(code, {filename: 'webppl:eval'}).code;
  return eval.call(global, compiledCode)({})(runner);
};

function getParamSync(nh, param){
  var path = ''
  var isFinished = false

  nh.getParam(param).then((val) => {
    path = val;
    isFinished = true;
  });

  

  return path
}

function init(rosNode, path){

  console.log(">>>>>>>"+path)
  

  var modelCode = fs.readFileSync(path+'model_vicpark.wppl', 'utf8');
  var loopCode = fs.readFileSync(path+'loop.wppl', 'utf8');

  let modelObject = getCompiledObject(modelCode)
  let modelOutput = evaluate(modelObject, globalStore);
  extend(globalStore, modelOutput.s);

  let loopObject = getCompiledObject(loopCode)

  globalStore.records = {}

  for(let readingName in globalStore.subs) {
    // Handling the message type
    let valueName = globalStore.subs[readingName];

    registerValue(rosNode, valueName, (data)=>{
      // globalStore.readings[readingName] = data;
      //console.log("received " + data);
      if(globalStore.posterior)
        globalStore.prior = globalStore.posterior;
      delete globalStore.posterior;
      globalStore.readings = {timestamp_s: Date.now()/1000};
      globalStore.readings[readingName] = data;

      globalStore._sumW = 0.
      globalStore._numFactorCalls = 0.

      global.count = 0.
      
      let result = evaluate(loopObject, globalStore);
      globalStore = result.s;

      // console.log("sumW = " + s._sumW)
      // console.log("numFactorCalls = " + s._numFactorCalls)


      // globalStore.posterior = result.k;

      for(const key in globalStore.actions){
          const publish = globalStore.pubs[key]
          
          if(typeof publish == "function"){
            // console.log(publish+"")
            publish(globalStore.actions[key])
          }
      }

      // console.log(globalStore);
      // console.log(globalStore.records)
    })
  }

  for(let readingName in globalStore.reps) {
    let valueName = globalStore.reps[readingName];

    registerValue(rosNode, valueName, (data)=>{
      globalStore.records[readingName] = data
    })
  }

  for(const actionName in globalStore.pubs) {
    // Handling the message type
    const valueName = globalStore.pubs[actionName];

    tryAdvertiseValue(rosNode, valueName, (publish)=>{
      globalStore.pubs[actionName] = publish
    })
  }
}


function main() {
  // Register node with ROS master
  rosnodejs.initNode('/rosppl_node')
    .then((nh) => {
      nh.getParam('webppl_path').then((path) => {
        init(nh, path)
      });
    });    
}

if (require.main === module) {
  // Invoke Main Listener Function
  main();
}
