# iiwapy
A Python interface to the [KUKA LBR IIWA R800/R820](https://www.kuka.com/en-gb/products/robotics-systems/industrial-robots/lbr-iiwa) through the [iiwa_stack](https://github.com/IFL-CAMP/iiwa_stack).

#### Requirements
* Python 3
* ROS Noetic
* [iiwa_stack](https://github.com/IFL-CAMP/iiwa_stack)
* KUKA IIWA

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

To run the examples be sure to set the correct namespace for your robot. By default this is `iiwa`. You can achieve this using `rosrun` or setting the environment variable `ROS_NAMESPACE` before launching your python script.

```python
import os
os.environ['ROS_NAMESPACE'] = 'iiwa'  # Needs to happen before rospy import
import rospy
rospy.init_node("synopsis")
```

### Basic motion

Move to a joint position, note this calls KUKA's PTP joint position API on the controller.

```python
from iiwapy import MotionController

control = MotionController()
promise = control.move_to_joint_position([-0.17, 1.0, 0.14, -1.75, -0.18, -1.19, -0.21])
promise.wait_for_result(60)  # seconds
```

Get the current pose (world frame):

```python
pose = control.get_pose() # PoseStamped
```

Make a linear move 10cm in X, note this calls a PTP LIN motion on the controller

```python
from copy import deepcopy

target = deepcopy(pose)
target.pose.position.x += 0.1
control.move_to_pose_lin(target, velocity=0.5, acceleration=0.5).wait_for_result(10)
```

Make a PTP motion

```python
target2 = deepcopy(pose)
target2.pose.position.z += 0.1
control.move_to_pose(target2, velocity=0.5, acceleration=0.5).wait_for_result(10)
```

### Switch control modes

Activate desired force mode applying 1 newton along X

```python
from iiwapy import DOF, Damping, Stiffness, control_mode

control_mode.desired_force(DOF.X, force=1, stiffness=1000)
```

Impedance control mode, additional documentation provided for `Stiffness` and `Damping`:

```python
control_mode.cartesian_impedance(Stiffness(), Damping())
```

Position control (default):

```python
control_mode.position()
```

### Media Flange IO

Access to the IO pins on the media flange (tested on [pneumatic media flange](https://www.kuka.com/-/media/kuka-downloads/imported/9cb8e311bfd744b4b0eab25ca883f6d3/kuka_sensitiverobotics_lbriiwa_insert_en.pdf?rev=aeb9b863fcde45f6b8d24f0f856f9803&hash=E73F5B2661E711F8A460560FA7E2199E))

```python
from iiwapy import MediaFlange

flange = MediaFlange()
outputs = flange.get_outputs()
print(outputs)

inputs = flange.get_inputs()

if inputs.input_1:
    flange.set_outputs(output_2=True)
```

### Notes

[JupyterLab](https://jupyterlab.readthedocs.io/en/stable/) is ideal for experimenting with the robot, make sure you have workspace monitoring and completed [safety configuration](https://github.com/IFL-CAMP/iiwa_stack/wiki/safetyconf) before hand though.

----

### Acknowledgements
This repository is part of a sub-project that has indirectly received funding from the European Union's Horizon 2020 research and innovation programme via an Open Call issued and executed under project TRINITY (grant agreement No 825196).   
<img src="https://europa.eu/european-union/sites/europaeu/files/docs/body/flag_yellow_low.jpg" width="150" />       ![TRINITY](https://trinityrobotics.eu/wp-content/uploads/2019/10/trinity-logo.svg)
