# ros-slam-nav

### 1. Installation
```
cd ~/catkin_ws/src/
git clone https://github.com/thanhphong98/ros-slam-nav
cd catkin_ws
catkin_make
```

### 2. Simulation
 
#### 2.1. World simulation
There are many virtual environments that availabled in robot_gazebo's launch folder. Run below command to open virtual house environment. 
```
roslaunch robot_gazebo robot_house.launch
```

In order to teleoperate the robot with the keyboard,  launch the teleoperation node with below command in a new terminal window.
```
roslaunch robot_teleop robot_teleop_key.launch
```

#### 2.2. SLAM simulation
The following instructions require prerequisites from the previous section. 
Open a new terminal window from the PC and run  the SLAM node. Hector SLAM method is used by default. 
```
roslaunch robot_slam robot_slam.launch slam_methods:=gmapping
```
When the map is created successfully, open a new terminal from remote PC and save the map.
```
rosrun map_server map_saver -f ~/map
 ``` 
#### 2.3. Navigation simulation
Just like the SLAM simulation, Navigation simulation also requires prerequisites from world simulation section. 
Open a new terminal and run the Navigation node.
```
roslaunch robot_navigation robot_navigation.launch
``` 
 
### 3. SLAM
Run roscore from remote PC. 
```
roscore 
```
 
Open a new terminal from Single Board Computer (SBC) on the robot and launch the Bringup. 
```
roslaunch robot_bringup robot.launch
```

Open a new terminal from remote PC and launch the SLAM node. 
```
roslaunch robot_slam robot_slam.launch
```
Run the teleoperation node to control the robot. 
```
roslaunch robot_teleop robot_teleop_key.launch 
```
Save the map. 
```
rosrun map_server map_saver -f ~/map
```
 
### 4. Navigation
Run roscore from remote PC. 
```
roscore 
```
 
Open a new terminal from Single Board Computer (SBC) on the robot and launch the Bringup. 
```
roslaunch robot_bringup robot.launch
```

Open a new terminal from remote PC and launch the Navigation node. 
```
roslaunch robot_navigation robot_navigation.launch
```

### 5. Problem
For any problem, please report to [Issues](https://github.com/thanhphong98/ros-slam-nav/issues) or contact email thanhphongho1998@gmail.com 
 
 
 



