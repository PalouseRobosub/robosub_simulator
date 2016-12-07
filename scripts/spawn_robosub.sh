#!/bin/bash

rosrun gazebo_ros spawn_model -file $(rospack find robosub_simulator)/models/cobalt/model.sdf -sdf -x 0 -y 0 -z 0.1 -model robosub
