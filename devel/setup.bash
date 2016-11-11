# Source this file from your bashrc
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$(rospack find simulator_gazebo)/models
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$(rospack find simulator_gazebo)/../../devel/lib
