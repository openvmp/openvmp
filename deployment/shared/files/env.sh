source /opt/ros/humble/setup.bash
source /root/openvmp/platform/install/local_setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/root/.openvmp/fastdds_client.xml
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export ROBOT_ID=$(cat /root/.openvmp/id)