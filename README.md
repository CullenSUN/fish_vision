# fish_vision

We are building the vision system for a very fast robotic fish. Target to avoid obstacles with the vision. 

## Hardware
- Raspberry Pi 4 (Model B 8GB RAM)
- Raspberry Pi Camera V2

## Software
- Ubuntu 16.04 + ROS Kinetic, image can be downloaded from [Ubiquity Robotics](https://downloads.ubiquityrobotics.com).
- Python 2.7.12
- OpenCV 3.3.1
- Numpy 1.11.3

## How to run
### tools
- "video_capture.py" can be run in Raspberry Pi for capturing videos continuously.

### Playgroun
"playground" folder contains code that has no hardware or OS dependency. Once set up environment with Python 2.7.X, OpenCV 3.3.1, Numpy 1.11.3, you shall be able to run them.   
### ROS nodes
1. ```roslaunch raspicam_node camerav2_1280x720.launch``` in Raspberry Pi.
2. ```rosrun obstacle_detector_node obstacle_detector.py``` in Raspberry Pi.
3. ```rosrun obstacle_avoidance_node obstacle_avoidance.py``` in Raspberry Pi.
4. ```rosrun debug_node vision_debugger.py``` in another computer.
