# Ned ROS stack

Licensed under GPLv3 (see [LICENSE file](LICENSE))

This repository contains all ROS packages used on Ned robot (Raspberry Pi 4B - Ubuntu server 20.04 for ARM).
## Ned ROS Stack overview

Here's a global overview of Ned ROS Stack :

![Ned ros stack - global overview](docs/.static/images/ROS_stack_overview.png)

## How to use Ned with a graphical interface ?

You can [download Niryo Studio](https://niryo.com/niryostudio/) (Linux, Windows and MacOS compatible).

## Documentation link

You can find the whole generated documentation of this project at [Niryo Documentation](https://niryorobotics.github.io/ned_ros/)

# My additions

I have added the gesture_control package to this repo that I have forked from Niryo Robotics' main ned_ros.

The package should theoretically compile, because the Python code works properly. 

I have fixed Gazebo to open without using niryo_robot_vision, as it did not allow the gesture control package to use the cameras. It is also possible to use this with DroidCam or any other webcam. 
All code work and other for the gesture control package has been done by me and no-one else (apart from the developers who worked on the actual repository).


