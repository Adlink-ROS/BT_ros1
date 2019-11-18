## BT_ros1
Behavior Tree example for ROS1

# Installation
1. git clone this repo into your ROS workspace. (default: catkin_ws)
```
cd ~/catkin_ws/src
git clone https://github.com/Adlink-ROS/BT_ros1.git
```

2. Install dependencies with my favorite command line
```
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Install SEMA ( For those devices do not need SEMA, just simply delete all lines related to "sema". )

4. Compile the workspace
```
cd ~/catkin_ws
catkin_make
```

# Test 

## Prerequisite
In this test, we will use turtlebot3 as the platform. Therefore, you need to install ros1 and Gazebo first.
```
sudo apt install ros-melodic-desktop-full ros-melodic-turtlebot3 ros-melodic-turtlebot3-gazebo
```

## Generate the map

### Terminal 1: Run Gazebo Emulator
```
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

### Terminal 2: Run SLAM
```
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_slam turtlebot3_slam.launch
```

### Terminal 3: Teleop
```
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
Then, try to use your perfect control skill to map the map. 

### Terminal 5 Save map
```
rosrun map_server map_saver -f ~/sim_map
```

## Use BT for navigation

### Terminal 1: Run Gazebo Emulator
```
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

### Terminal 2: Run navigation
```
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/sim_map.yaml
```

### Terminal 3: Run BT
```
source devel/local_setup.bash
rosrun bt_sample node _file:=absolute_path/bt_test.xml
```

TODO: Put some pics
