
alias load_ros2="source /opt/ros/humble/setup.bash"
alias load_microros="source ~/Ros2_esp32/install/local_setup.bash"
alias load_microagent="source ~/uros_ws/install/local_setup.bash"
alias load_fg="source ~/Ros2_fg_prj/install/setup.bash"

ros2 run joy joy_node
ros2 run fg_bridge Ros2_main
ros2 run fg_bridge Ros2fg_bridge
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM* -b 115200
