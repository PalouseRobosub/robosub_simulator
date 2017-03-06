#!/bin/bash
# this script will jump the sub
rosservice call /gazebo/set_model_state "model_state:
  model_name: 'robosub'
  pose:
    position:
      x: 27.37
      y: -4.24
      z: -2.92
    orientation:
      x: 0.0
      y: 0.0
      z: 0.14
      w: 0.99"
rostopic pub --once /control robosub/control "
forward_state: 3
strafe_state: 3
dive_state: 1
roll_state: 1
pitch_state: 1
yaw_state: 1
forward: 0.0
strafe_left: 0.0
dive: -2.12
roll_right: 0.0
pitch_down: 0.0
yaw_left: 16.2"
