# Experimental Robotics Laboratory - Assignment 2 ROS PLANNING AND SLAM

In this project a mobile robot is planned to visit 4 waypoints in the Gazebo world, And goes to the waypoint with the Aruco-marker that has smallest id. It uses PlanSys2 for planning, Navigation2 for navigation, and detection of Aruco markers using the ros2 Aruco library. 

This project is developed by:
1. *Mehdi Raza Khorasani - s6164555*
2. *Ouassim Milous - s5938924*
3. *Younes Hebik - s5813030*
4. *Ozan Pali - s5831146*

## Demo
Watch our demo video below:

  [![Watch the video](https://img.youtube.com/vi/YxPhdKM9SUE/hqdefault.jpg)](https://www.youtube.com/watch?v=YxPhdKM9SUE)

## Dependencies

1. **PlanSys2:**    - [PlanSys2 GitHub Repository](https://github.com/PlanSys2/ros2_planning_system/tree/humble-devel)


2. **Slam Toolbox** - [Slam Toolbox Github Repository](https://github.com/SteveMacenski/slam_toolbox/tree/humble)

3. **Navigation2**  - [Navigation2 Github Repository](https://github.com/ros-navigation/navigation2/tree/humble)

4. **Aruco:**  - [OpenCV Aruco Marker Tracking GitHub Repository](https://github.com/carmineD8/ros2_aruco)


## Installation
docker-compose.yaml file is going to be used to build the environment. 


- First of all create a folder in your local machine to be mounted inside docker container
```
mkdir docker
```
- Create the workspace for the project 
```
cd docker
mkdir -r ros_ws/src
cd ros_ws/src
```
- Clone the packages required for the project inside src folder
```
git clone https://github.com/OuassimMilous/ERL-assignment2-ROS-PLANNING-AND-SLAM
git clone https://github.com/SteveMacenski/slam_toolbox/tree/humble
git clone https://github.com/carmineD8/ros2_aruco
git clone https://github.com/PlanSys2/ros2_planning_system/tree/humble-devel
```

- Build container with the yaml file inside folder that you created(docker)
```
mv /ERL-assignment2-ROS-PLANNING-AND-SLAM/docker-compose.yaml ../../../
mv /ERL-assignment2-ROS-PLANNING-AND-SLAM/env-setup.sh ../../../
cd ../../../
docker compose up -d
```
- Start the container and do the next instructions inside container
```
docker exec -it exp-assignment-2 bash
```
- Install the navigation package with apt
```
apt update
cd /data
chmod +x env-setup.sh
./env-setup.sh
```

- Install the dependencies of the packages and build your workspace
```
cd /data/ros_ws
colcon build
```

## Running the project

- In the first terminal: Source your workspace and launch the required python file
```
ros2 launch autonomous_planner assignement2_launch.py
```
- In the new terminal: Source your workspace and start the action server node
```
ros2 run autonomous_planner controller_node 
```
Note: For the move action and the controller node were inspired by ros2_planning_system_examples repository of PlanSys2 [Click  to see utilized part](https://github.com/PlanSys2/ros2_planning_system_examples/tree/humble/plansys2_patrol_navigation_example/src)   

