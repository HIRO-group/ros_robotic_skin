gnome-terminal -e 'roscore'
gnome-terminal -e "ssh -t rp14 'source ~/.bashrc; source /opt/ros/melodic/setup.bash; export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/hiro/catkin_ws; sleep 5; cd catkin_ws/src/ros_robotic_skin/scripts/publishers/; python accelerometer_publisher.py; exec bash'"
gnome-terminal -e "ssh -t rp14 'source ~/.bashrc; source /opt/ros/melodic/setup.bash; export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/hiro/catkin_ws; sleep 5; cd catkin_ws/src/ros_robotic_skin/scripts/publishers/; python accelerometer_publisher.py; exec bash'"
gnome-terminal -e "ssh -t rp23 'source ~/.bashrc; source /opt/ros/melodic/setup.bash; export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/hiro/catkin_ws; sleep 5; cd catkin_ws/src/ros_robotic_skin/scripts/publishers/; python accelerometer_publisher.py; exec bash'"
gnome-terminal -e "ssh -t rp23 'source ~/.bashrc; source /opt/ros/melodic/setup.bash; export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/hiro/catkin_ws; sleep 5; cd catkin_ws/src/ros_robotic_skin/scripts/publishers/; python accelerometer_publisher.py; exec bash'"
gnome-terminal -e "ssh -t rp56 'source ~/.bashrc; source /opt/ros/melodic/setup.bash; export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/hiro/catkin_ws; sleep 5; cd catkin_ws/src/ros_robotic_skin/scripts/publishers/; python accelerometer_publisher.py; exec bash'"
gnome-terminal -e "ssh -t rp56 'source ~/.bashrc; source /opt/ros/melodic/setup.bash; export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/hiro/catkin_ws; sleep 5; cd catkin_ws/src/ros_robotic_skin/scripts/publishers/; python accelerometer_publisher.py; exec bash'"