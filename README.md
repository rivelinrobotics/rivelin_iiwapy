# iiwapy
A Python interface to the [KUKA LBR IIWA R800/R820](https://www.kuka.com/en-gb/products/robotics-systems/industrial-robots/lbr-iiwa) through the [iiwa_stack](https://github.com/IFL-CAMP/iiwa_stack).

## Installation

First install the [iiwa_stack](https://github.com/IFL-CAMP/iiwa_stack), if you want to make use of the pneumatic media flange IO pins consider using our [fork](https://github.com/additiveautomations/iiwa_stack) or cherry picking [691147](https://github.com/IFL-CAMP/iiwa_stack/commit/6911477e3a492ce7ff25dde5574095ff1f212ce1) and [b73a00](https://github.com/IFL-CAMP/iiwa_stack/commit/b73a00b5801bdaa88a4e27b129450268d4dc7631)

Create a new [Catkin](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) workspace:

```
$ source /opt/ros/noetic/setup.bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
```

Clone iiwpy into the workspace:
```
$ git clone git@github.com:additiveautomations/iiwapy.git
```

Build and install
```
$ catkin_make install
```

## Synopsis

