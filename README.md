# drone-gazebo-sim
This is a Gazebo simulation package for ros 18.04. The package is migrated from the [rotorS](https://github.com/ethz-asl/rotors_simulator).

# Requirements
* Ubuntu 18.04 ros-melodic
* gazebo greater than 9.0

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

```
roslaunch rotors_gazebo firefly_swarm_hovering_example.launch 
roslaunch rotors_gazebo controller.launch 
```
