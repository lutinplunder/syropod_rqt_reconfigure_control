# Syropod RQT Reconfigure Control

[![Syropod Banner](https://i.imgur.com/QyMTwG3.jpg "CSIRO Robotics")](https://research.csiro.au/robotics/)

An interface for user input via RQT Reconfigure Graphical User Interface (GUI) to [Syropod Highlevel Controller](https://github.com/csiro-robotics/syropod_highlevel_controller).

Note that [syropod_remote](https://github.com/csiro-robotics/syropod_remote) cannot be running when using syropod_rqt_reconfigure_control.

## Getting Started

If you haven't looked at the tutorials for using Syropod Highlevel Controller, see [SHC Tutorials](https://github.com/csiro-robotics/shc_tutorials).

### Requirements

* Ubuntu 18.04 LTS
* ROS Melodic

### Dependencies

* [Syropod High-level Controller](https://github.com/csiro-robotics/syropod_highlevel_controller):
  * `git clone https://github.com/csiro-robotics/syropod_highlevel_controller.git`

### Installation

```bash
cd catkin_ws/src
git clone https://github.com/csiro-robotics/syropod_rqt_reconfigure_control.git
cd ..
catkin build
```

## Operation

### Control Commands

* System State - Controls SHC system state. Select the system state to be Operational/Suspended from the drop down menu.

* Robot State - Controls the current robot state. Select the robot state to be Packed/Ready/Running from the drop down menu.

* Linear X - Adjust the linear velocity of the hexapod in X direction using the slider from -1 to 1.

* Linear Y - Adjust the linear velocity of the hexapod in Y direction using the slider from -1 to 1.

* Angular Z - Adjust the rotational velocity of the hexapod around Z axis using the slider from -1 to 1.

* Stop - Click on the tick box to stop the hexapod when moving. Click again to start moving the hexapod.

* Gait Selection - Controls the gait designation of the hexapod. Select the gait from Wave/Amble/Ripple/Tripod gaits using the drop down menu.

* Cruise Control Mode - Select the cruise control mode to be ON/OFF from the drop down menu.

## Nodes

### rqt_control

#### Subscribed Topics

* Nil

#### Published Topics

* System State:
  * Description: The desired state of the entire Syropod High-level Controller system.
  * Topic: */syropod\_remote/system\_state*
  * Type: std_msgs::Int8
* Robot State:
  * Description: The desired state of the robot.
  * Topic: */syropod\_remote/robot_state*
  * Type: std_msgs::Int8
* Desired Velocity:
  * Description: The desired body velocity of the robot.
  * Topic: */syropod\_remote/desired\_velocity*
  * Type: geometry_msgs::Twist
* Gait Selection:
  * Description: The desired gait selection for the walk controller of the robot.
  * Topic: */syropod\_remote/gait\_selection*
  * Type: std_msgs::Int8
* Cruise Control Mode:
  * Description: The desired cruise control mode.
  * Topic: */syropod\_remote/cruise\_control\_mode*
  * Type: std_msgs::Int8

## Changelog

* v0.1.0
  * Initial version
  * Control: system state; robot state; linear x/y and angular z; STOP toggle, gait selection; and cruise control mode.

## Authors

* Oshada Jayasinghe
* Benjamin Tam

## License

This project is licensed under the CSIRO Open Source Software Licence Agreement (variation of the BSD / MIT License) - see the [LICENSE](LICENSE) file for details.

## Issues

Please report bugs using [Issue Tracker](https://github.com/csiro-robotics/syropod_rqt_reconfigure_control/issues) or contact us via email [shc-support@csiro.au](mailto:shc-support@csiro.au).
