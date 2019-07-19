# drone-gazebo-sim
This is a Gazebo simulation package for ros 18.04. The package is migrated from the [rotorS](https://github.com/ethz-asl/rotors_simulator).

# Requirements
* Ubuntu 18.04 ros-melodic
* gazebo greater than 9.0

```
sudo apt-get install ros-melodic-desktop-full ros-melodic-joy ros-melodic-octomap-ros ros-melodic-mavlink python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-melodic-control-toolbox
sudo apt-get install python-cvxopt
//or
sudo pip install cvxopt
```
# Additional package

```
sudo apt-get install ros-melodic-ompl
sudo apt-get install ros-melodic-mavros
sudo apt-get install ros-melodic-mavros-extras 
sudo apt-get install ros-melodic-mavros-msgs
sudo apt-get install libompl-dev
cd /opt/ros/melodic/lib/mavros
sudo ./install_geographiclib_datasets.sh


```

# Compiling
download the package and put it into workspace and use `catkin_make` to build the package.
If the workspace is not ready than try the following command:
```
cd ~/
mkdir catkin_ws && cd catkin_ws/src
git clone https://github.com/EdXian/drone-gazebo-sim.git
cd ..
catkin_make
```
# Running

## Trajecotry tracking
```
roslaunch rotors_gazebo firefly_swarm_hovering_example.launch 
roslaunch rotors_gazebo controller.launch 
roslaunch ukf leader_follower_force_estimate.launch
rosparam set /force_control true
rosparam set /start true

```
## Path planning
```

```




