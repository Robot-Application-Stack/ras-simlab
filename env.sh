source /opt/ros/humble/setup.bash
export RAS_APP_PATH=$(realpath $(dirname "${BASH_SOURCE[0]}"))
export RAS_WORKSPACE_PATH=$RAS_APP_PATH/ros2_ws
export IGN_GAZEBO_RESOURCE_PATH=$RAS_APP_PATH/assets/objects/old_models:$RAS_APP_PATH/third_party/gazebo_models:${IGN_GAZEBO_RESOURCE_PATH}
source $RAS_WORKSPACE_PATH/install/setup.bash
