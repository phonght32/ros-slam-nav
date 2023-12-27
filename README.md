# ros-slam-nav
This project based on TurtleBot3

### 1. Installation
```
cd ~/ros_ws/src/
git clone https://github.com/phonght32/ros-slam-nav
cd ros_ws
colcon build
```

### 2. Simulation

#### 2.1. World simulation
There are many virtual environments that availabled in robot_gazebo's launch folder. Run below command to open virtual house environment. 
```
ros2 launch robot_gazebo robot_world.launch.py
```

In order to teleoperate the robot with the keyboard,  launch the teleoperation node with below command in a new terminal window.
```
ros2 run robot_teleop robot_teleop
```

#### 2.2. SLAM simulation
Open a new terminal window from the PC and run the SLAM node.
```
ros2 launch robot_slam cartographer.launch.py use_sim_time:=True
```
When the map is created successfully, open a new terminal from remote PC and save the map.
```
ros2 run nav2_map_server map_saver_cli -f ~/map
```
#### 2.3. Navigation simulation
Open a new terminal and run the Navigation node.
```
ros2 launch robot_navigation navigation.launch.py use_sim_time:=True
```

### 3. SLAM

### 4. Navigation

### 5. Problem

For any problem, please report to [Issues](https://github.com/phonght32/ros-slam-nav/issues) or contact email thanhphongho1998@gmail.com 

 



