
Terminal #1:
cd turtle_mpc
singularity shell mpc.sif
source /opt/ros/noetic/setup.bash 
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch


----------------------------------------------------------------------------
Terminal #2:
cd turtle_ros_bridge
singularity shell --writable ros_bridge_sandbox/
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash
cd bridge
source install/setup.bash
rosparam load bridge.yaml
ros2 run ros1_bridge parameter_bridge


----------------------------------------------------------------------------
Terminal #3:
cd turtle_mpc
singularity shell mpc.sif
source /opt/ros/noetic/setup.bash 
cd mpc_ros/
source devel/setup.bash 
rosrun mpc_ros nmpc_node 



----------------------------------------------------------------------------
Terminal #4:
cd turtle_llm
singularity shell --writable turtlebot3_agent_sandbox/
source /opt/ros/humble/setup.bash
cd llm_unit
source install/setup.bash
export GOOGLE_API_KEY="  "
ros2 topic list
ros2 run turtlebot3_agent main



export ROS_DOMAIN_ID=42

