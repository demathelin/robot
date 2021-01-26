[![pipeline status](https://gitlab.inria.fr/auctus/panda/velocity_qp/badges/master/pipeline.svg)](https://gitlab.inria.fr/auctus/panda/velocity-qp/commits/master)
[![Quality Gate](https://sonarqube.inria.fr/sonarqube/api/badges/gate?key=auctus:panda:velocity-qp)](https://sonarqube.inria.fr/sonarqube/dashboard/index/auctus:panda:velocity-qp)
[![Coverage](https://sonarqube.inria.fr/sonarqube/api/badges/measure?key=auctus:panda:velocity-qp&metric=coverage)](https://sonarqube.inria.fr/sonarqube/dashboard/index/auctus:panda:velocity-qp)

[![Bugs](https://sonarqube.inria.fr/sonarqube/api/badges/measure?key=auctus:panda:velocity-qp&metric=bugs)](https://sonarqube.inria.fr/sonarqube/dashboard/index/auctus:panda:velocity-qp)
[![Vulnerabilities](https://sonarqube.inria.fr/sonarqube/api/badges/measure?key=auctus:panda:velocity-qp&metric=vulnerabilities)](https://sonarqube.inria.fr/sonarqube/dashboard/index/auctus:panda:velocity-qp)
[![Code smells](https://sonarqube.inria.fr/sonarqube/api/badges/measure?key=auctus:panda:velocity-qp&metric=code_smells)](https://sonarqube.inria.fr/sonarqube/dashboard/index/auctus:panda:velocity-qp)

[![Line of code](https://sonarqube.inria.fr/sonarqube/api/badges/measure?key=auctus:panda:velocity-qp&metric=ncloc)](https://sonarqube.inria.fr/sonarqube/dashboard/index/auctus:panda:velocity-qp)
[![Comment ratio](https://sonarqube.inria.fr/sonarqube/api/badges/measure?key=auctus:panda:velocity-qp&metric=comment_lines_density)](https://sonarqube.inria.fr/sonarqube/dashboard/index/auctus:panda:velocity-qp)

# Links
- Sonarqube : https://sonarqube.inria.fr/sonarqube/dashboard?id=auctus%3Apanda%3Avelocity-qp
- Documentation : https://auctus.gitlabpages.inria.fr/panda/velocity_qp/index.html



# Velocity QP

A generic low-level joint velocity controller with a QP formulation.

## Some necessary package to install first:

`sudo apt install python-rosdep python-catkin-tools python-wstool python-vcstool build-essential cmake git libpoco-dev libeigen3-dev ros-melodic-combined-robot-hw`


## Install libfranka
1. `mkdir -p ~/franka_ros_ws`
2. `git clone --recursive https://github.com/frankaemika/libfranka ~/franka_ros_ws/libfranka`
3. `cd ~/franka_ros_ws/libfranka`
4. `git checkout 0.7.1`
5. `git submodule update`
6. `mkdir build; cd build`
7. `cmake -DCMAKE_BUILD_TYPE=Release ..`
8. `cmake --build .`


## Install franka_ros 
1. `cd ~/franka_ros_ws`
2. `git clone --recursive https://github.com/frankaemika/franka_ros src/franka_ros`
3. `cd src/franka_ros`
4. `git checkout melodic-devel` 
5. `cd ~/franka_ros_ws`
6. `catkin config --init --extend /opt/ros/melodic --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-std=c++11 -DFranka_DIR:PATH=~/franka_ros_ws/libfranka/build`
7. `catkin build`
8. `source ~/franka_ros_ws/devel/setup.bash`


## Install torque_qp
1. Configure a catkin workspace that extends the `franka_ros_ws`.
    -   `mkdir -p ~/auctus_ws/src`
    -   `cd ~/auctus_ws`
    -   `catkin config --init --extend ~/franka_ros_ws/devel --cmake-args -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=~/franka_ros_ws/libfranka/build -DCMAKE_CXX_FLAGS=-std=c++11`
2. `cd ~/auctus_ws/src`
3. `git clone https://gitlab.inria.fr/auctus/panda/velocity_qp.git`
4. `wstool init `
5. `wstool merge velocity_qp/velocity_qp.rosinstall`
6. `wstool update`
7. `cd ..`
8. `rosdep install --from-paths src --ignore-src -r -y --skip-keys="libfranka"`
9. `catkin build`

## Change environment variables

1. Add in your bashrc `source ~/catkin_ws/devel/setup.bash`
2. Add in your bashrc `export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-9:~/auctus_ws/src/franka_description/worlds` and `export GAZEBO_MODEL_PATH=~/auctus_ws/src/franka_description/robots`
3. If its the first you install gazebo launch it: `gazebo`. Wait for it to launch then close it.

# Test your installation

To test your installation go to [Usage](https://gitlab.inria.fr/auctus/panda/torque_qp/-/wikis/Usage)

# Usage

In simulation 

`roslaunch velocity_qp run.launch sim:=true`

On the real robot

`roslaunch velocity_qp run.launch robot_ip:=your_robot_ip`

Load a new trajectory online (stored in panda_traj/trajectories/name_of_the_trajectory.csv). This uses a ros service to update the trajectory.

`rosrun velocity_qp load_trajectory.py name_of_the_trajectory`

For example : 

`rosrun velocity_qp load_trajectory.py back_and_forth`

Play the trajectory (you can also use rqt to call the service) This uses a ros service to interact with the code.

`rosservice call /velocity_qp/updateUI "play_traj : true"`

Publish the trajectory 

`rosservice call /velocity_qp/updateUI "publish_traj : true"`

## Optional roslaunch parameter : 

`robot_ip` IP adress of the panda robot

`load_gripper` if **True** the gripper is loaded in the xacro descritpion file and the gripper action server is instantiated.

`sim` Run the code in simulation on Gazebo

# Controller parameters

The controller parameters are stored in a yaml file in the `/config` folder and loaded as ros parameters in the `run.launch` file. They are then read by the 
controller with load_parameters(); .

# Custom messages and service

Custom messages can be defined in the `msg` folder. An example is provided with the `PandaRunMsg.msg` file.
Custom services can be defined in the `srv` folder. An example is provided with the `UI.srv` file.
