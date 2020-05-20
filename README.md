# BT_ros1

Behavior Tree example for ROS1

# Build up environment

1. git clone the repo.
```
mkdir -p ~/ros1_bt_ws/src
cd ~/ros1_bt_ws/src
git clone https://github.com/Adlink-ROS/BT_ros1.git
```

2. Install dependencies
```
cd ~/ros1_bt_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build
```
catkin_make
```

# Usage

We use [NeuronBot2](https://github.com/Adlink-ROS/neuronbot2/tree/melodic-devel) as example.
We will run Gazebo with NeuronBot2 and show a simple BT example.

The BT example (refer to [bt_nav_mememan_interrupt.xml](bt_xml/bt_nav_mememan_interrupt.xml)) will make NeuronBot2 move between Goal_a and Goal_b.
If receiving `/interrupt_event`, which is `gohome`, then NeuronBot2 will move to Goal_c.

* Open 1st terminal and run mememan world. (melodic environment)
```
source ~/neuronbot2_ros1_ws/devel/setup.bash
export GAZEBO_MODEL_PATH=~/neuronbot2_ros1_ws/src/neuronbot2/neuronbot2_gazebo/models
roslaunch neuronbot2_gazebo neuronbot2_world.launch world_model:=mememan_world.model
```
* Open 2nd terminal and run navigation. (melodic environment)
```
source ~/neuronbot2_ros1_ws/devel/setup.bash
roslaunch neuronbot2_nav neuronbot2_nav.launch map_name:=$HOME/neuronbot2_ros1_ws/src/neuronbot2/neuronbot2_nav/maps/mememan.yaml open_rviz:=true
```
* Open 3rd termainal and run BT. (melodic environment) 
```
source ~/ros1_bt_ws/devel/setup.bash
rosrun bt_sample node _file:=$HOME/ros1_bt_ws/src/BT_ros1/bt_xml/bt_nav_mememan_interrupt.xml
```
* Open 4th terminal and pub interrupt event. (melodic environment)
```
rostopic pub /interrupt_event std_msgs/String "gohome"
```
