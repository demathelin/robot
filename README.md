[![pipeline status](https://gitlab.inria.fr/auctus/panda/velocity_qp/badges/master/pipeline.svg)](https://gitlab.inria.fr/auctus/panda/velocity_qp)
[![Quality Gate Status](https://sonarqube.inria.fr/sonarqube/api/project_badges/measure?project=auctus%3Apanda%3Avelocity-qp&metric=alert_status)](https://sonarqube.inria.fr/sonarqube/dashboard?id=auctus%3Apanda%3Atorque-qp)
[![Coverage](https://sonarqube.inria.fr/sonarqube/api/project_badges/measure?project=auctus%3Apanda%3Avelocity-qp&metric=coverage)](https://sonarqube.inria.fr/sonarqube/dashboard?id=auctus%3Apanda%3Avelocity-qp)

# Links
- Sonarqube : https://sonarqube.inria.fr/sonarqube/dashboard?id=auctus%3Apanda%3Avelocity-qp
- Documentation : https://auctus.gitlabpages.inria.fr/panda/velocity_qp/index.html



# Velocity QP

A generic low-level joint velocity controller with a QP formulation.  
_Remark:_ I use ROS-Melodic and GAZEBO version 9.19.0 for Ubuntu 18.04.5 LTS.
## Some necessary package to install first:

`sudo apt install python-rosdep python-catkin-tools python-wstool python-vcstool build-essential cmake git libpoco-dev libeigen3-dev`

## Install franka_ros 
1. `mkdir ~/auctus_ws; cd ~/auctus_ws`
2. `git clone --recursive https://github.com/frankaemika/franka_ros franka_ros_ws/src`
3. `cd ~/auctus_ws/franka_ros_ws/src`
4. `git checkout $ROS_DISTRO-devel` 
5. `cd ~/auctus_ws/franka_ros_ws`
6. `catkin config --init --extend /opt/ros/$ROS_DISTRO --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-std=c++11`
7. `rosdep install --from-paths src --ignore-src -r -y`
8. `catkin build`
9. `source ~/auctus_ws/franka_ros_ws/devel/setup.bash`


## Install velocity_qp
1. Configure a catkin workspace that extends the `franka_ros_ws`.
    -   `mkdir -p ~/auctus_ws/dev_ros_ws/src`
    -   `cd ~/auctus_ws/dev_ros_ws/`
    -   `catkin config --init --extend ~/auctus_ws/franka_ros_ws/devel --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-std=c++11`
2. `cd ~/auctus_ws/dev_ros_ws/src`
3. `git clone https://gitlab.inria.fr/auctus/panda/velocity_qp.git`
4. `wstool init `
5. `wstool merge velocity_qp/velocity_qp.rosinstall`
6. `wstool update`
8. `rosdep install --from-paths ../src --ignore-src -r -y`
9. **New step**: Add `ENABLE_LANGUAGE{C}` in the file `velocity_qp\CMakeLists.txt` at the 9th line
10. **New step**: `cd ~/auctus_ws/dev_ros_ws/src`
11. `catkin build`

When I installed the first time the package, at the section "Install velocity_qp" at `catkin build`, I got an error. The compilator was not able to read a .c file. I checked on Internet to see if there was not a solution (I always check on Internet before asking to a colleague an help, I do not want to disturb them). And I saw that it lacked the line ENABLE_LANGUAGE(C) in the file CMakeLists.txt.

## Change environment variables

1.`echo "source ~/auctus_ws/dev_ros_ws/devel/setup.bash" >> ~/.bashrc`
2. `source ~/.bashrc`
3. If its the first you install gazebo launch it: `gazebo`. Wait for it to launch then close it.

# Usage

In simulation 

`roslaunch velocity_qp run.launch sim:=true`

![ALT](/image/step1.png)

On the real robot

`roslaunch velocity_qp run.launch robot_ip:=your_robot_ip`

Load a new trajectory online (stored in panda_traj/trajectories/name_of_the_trajectory.csv). This uses a ros service to update the trajectory.

`rosrun velocity_qp load_trajectory.py name_of_the_trajectory`

For example : 

`rosrun velocity_qp load_trajectory.py box`

![ALT](/image/step2.png)

Play the trajectory (you can also use rqt to call the service) This uses a ros service to interact with the code.

`rosservice call /velocity_qp/updateUI "play_traj : true"`

![ALT](/image/step3.png)

Publish the trajectory 

`rosservice call /velocity_qp/updateUI "publish_traj : true"`

## Video

A record is available in movie/arc.ogv
## Optional roslaunch parameter : 

`robot_ip` IP adress of the panda robot

`sim` Run the code in simulation on Gazebo

# Controller parameters

The controller parameters are stored in a yaml file in the `/config` folder and loaded as ros parameters in the `run.launch` file. They are then read by the 
controller with load_parameters(); .

![ALT](/image/step4.png)

# Custom messages and service

Custom messages can be defined in the `msg` folder. An example is provided with the `PandaRunMsg.msg` file.
Custom services can be defined in the `srv` folder. An example is provided with the `UI.srv` file.
