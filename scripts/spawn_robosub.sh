#!/bin/bash

rosrun gazebo_ros spawn_model -file $(rospack find robosub_simulator)/models/cobalt/model.sdf -sdf -x 7.0 -y -26.2 -z -1.2 -Y 0.8 -model robosub
