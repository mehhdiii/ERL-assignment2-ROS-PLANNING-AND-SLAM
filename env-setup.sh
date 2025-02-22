apt update && 
apt install ros-humble-xacro && 
apt install ros-humble-joint-state-publisher && 
apt install ros-humble-gazebo-ros2-control &&
apt install ros-humble-plansys2-domain-expert && 
apt install ros-humble-gtest-vendor && 
apt install ros-humble-rqt-gui-cpp && 
apt install ros-humble-rclcpp-cascade-lifecycle && 
apt install ros-humble-behaviortree-cpp-v3 && 
apt install ros-humble-test-msgs && 
apt install libreadline-dev && 
apt install ros-humble-nav2-msgs && 
apt install ros-humble-navigation2 -y && 
apt install ros-humble-nav2-bringup && 
apt install ros-humble-ros2-control && 
apt install ros-humble-ros2-controllers &&
apt install ros-humble-behaviortree-cpp && 
apt install ros-humble-plansys2-pddl-parser &&
apt install libsuitesparse-dev &&
apt install ros-humble-rviz2 && 
apt install ros-humble-gazebo-ros-pkgs &&
apt install '~nros-humble-rqt*' &&
cp -r /data/data/aruco_ros/aruco_ros/models /root/.gazebo/models &&
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc &&
echo 'source /data/ros_ws/install/setup.bash' >> ~/.bashrc