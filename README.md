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

# Installation
1. `sudo apt install python-wstool`
2. Follow the installation instruction of the fraka ros package : https://frankaemika.github.io/docs/installation_linux.html#installing-from-the-ros-repositories
3. (optional) Configure a catkin workspace that extends the previous workspace
4. `cd catkin_ws/src`
5. `git clone git@gitlab.inria.fr:auctus/panda/velocity_qp.git`
6. `wstool init `
7. If you have ssh-key setup : `wstool merge velocity_qp/velocity_qp_dep_ssh.rosinstall`. Else `wstool merge velocity_qp/velocity_qp_dep_https.rosinstall`
8. `wstool update`
9. `rosdep install --from-paths src --ignore-src -r -y`
10. `catkin build`
11. `source catkin_wd/devel/setup.bash`

# Usage

In simulation 

`roslaunch velocity_qp run.launch sim:=true`

On the real robot

`roslaunch velocity_qp run.launch robot_ip:=your_robot_ip`

Load a new trajectory online (stored in panda_traj/trajectories/name_of_the_trajectory.csv). This uses a ros service to update the trajectory.

`rosrun velocity_qp load_trajectory.py name_of_the_trajectory`

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

# Some installation requirement

`sudo apt install ros-melodic-track-ik`

`sudo apt install ros-melodic-eigen-conversions`

`sudo apt install ros-melodic-kdl-conversions`