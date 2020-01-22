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
5. `wstool init `
6. `wstool merge velocity_qp/velocity_qp_dep.rosinstall`
7. `wstool update`
8. `catkin build`

# Usage

`roslaunch velocity_qp run.launch`

## Optional parameter : 

`robot_ip` IP adress of the panda robot

`load_gripper` if **True** the gripper is loaded in the xacro descritpion file and the gripper action server is instantiated.

`sim` Run the code in simulation on Gazebo

`roslaunch velocity_qp run.launch sim:=true` in simulation mode


# Some installation requirement

`sudo apt install ros-melodic-track-ik`

`sudo apt install ros-melodic-eigen-conversions`

`sudo apt install ros-melodic-kdl-conversions`