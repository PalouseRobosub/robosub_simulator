# Source this file from your bashrc
source /usr/share/gazebo-7/setup.sh
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$(rospack find simulator_gazebo)/models
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$(rospack find simulator_gazebo)/../../devel/lib
export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:$(rospack find simulator_gazebo)/models
