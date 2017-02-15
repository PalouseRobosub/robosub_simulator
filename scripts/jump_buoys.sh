#!/bin/bash
# this script will jump the sub
rosservice call /gazebo/set_model_state "model_state:
  model_name: 'robosub'
  pose:
    position:
      x: 15.22
      y: -23.69
      z: -2.5
    orientation:
      x: 0.0
      y: 0.0
      z: 0.34
      w: 0.94"
rostopic pub --once /control robosub/control "
forward_state: 0
strafe_state: 0
dive_state: 1
roll_state: 1
pitch_state: 1
yaw_state: 1
forward: 0.0
strafe_left: 0.0
dive: -1.7
roll_right: 0.0
pitch_down: 0.0
yaw_left: 40.0"
