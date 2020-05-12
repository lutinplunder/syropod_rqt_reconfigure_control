# Syropod RQT Reconfigure Control

An interface between user input via RQT Reconfigure Graphical User Interface (GUI) and the Syropod High-level Controller (SHC).

Note that [syropod_remote](https://github.com/csiro-robotics/syropod_remote) cannot be running when using syropod_rqt_reconfigure_control.

## Inputs

* System State - Controls SHC system state. Select the system state to be Operational/Suspended from the drop down menu.

* Robot State - Controls the current robot state. Select the robot state to be Packed/Ready/Running from the drop down menu.

* Linear X - Adjust the linear velocity of the hexapod in X direction using the slider from -1 to 1.

* Linear Y - Adjust the linear velocity of the hexapod in Y direction using the slider from -1 to 1.

* Angular Z - Adjust the rotational velocity of the hexapod around Z axis using the slider from -1 to 1.

* Stop - Click on the tick box to stop the hexapod when moving. Click again to start moving the hexapod.

* Gait Selection - Controls the gait designation of the hexapod. Select the gait from Wave/Amble/Ripple/Tripod gaits using the drop down menu.

* Cruise Control Mode - Select the cruise control mode to be ON/OFF from the drop down menu.

## Changelog

* v0.1.0
  * Initial version
  * Control: system state; robot state; linear x/y and angular z; STOP toggle, gait selection; and cruise control mode.

## Author

Oshada Jayasinghe
