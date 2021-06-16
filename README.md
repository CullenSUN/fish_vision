# fish_vision

We are building the vision system for a very fast robotic fish. Target to avoid obstacles with the vision. 

## Hardware
- Raspberry Pi 4 (Model B 8GB RAM)
- Raspberry Pi Camera V2

## Software

- [Ubuntu 16.04](https://ubiquity-pi-image.sfo2.cdn.digitaloceanspaces.com/2020-11-07-ubiquity-xenial-lxde-raspberry-pi.img.xz)
- ROS Kinetic 
- Python 2.7.12
- OpenCV (cv2) 3.3.1

## How to run
- ```roslaunch raspicam_node camerav2_410x308_30fps.launch```
- ```rosrun obstacle_detector_node obstacle_detector.py```
- ```rosrun obstacle_avoidance_node obstacle_avoidance.py```
- ```rosrun debug_node vision_debugger.py```
