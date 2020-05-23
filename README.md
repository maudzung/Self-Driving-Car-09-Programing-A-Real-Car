# Programming a Real Self-Driving Car

---
The goal of the project to navigate safely a self-driving car around a course. The car needs to keep the lane, stop in front of red lights and obstacles.
The Robot Operative System (ROS) framework was used in the project. <br>

The general architecture of autonomous vehicles is described as below:

![architecture](./images/autonomous_vehicle_architecture.png)

## Demo

I used the provided ubuntu image from Udacity [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/Udacity_VM_Base_V1.0.0.zip) 
to run my code, and I used my Ubuntu host machine to run the simulator.

![demo](./images/demo.gif)
The full demo is at [https://youtu.be/lRTAEgUJmEw](https://youtu.be/lRTAEgUJmEw)

## Implementation details

The System Architecture Diagram
![System Architecture Diagram](./images/final-project-ros-graph-v2.png)

In this project, I have focused on 3 ROS nodes:

### 1. Waypoint Updater
The code of this node is in [./ros/src/waypoint_updater/waypoint_updater.py](./ros/src/waypoint_updater/waypoint_updater.py)
- The `waypoint_updater` node subscribes to the `/base_waypoints` topic, which publishes a list of all waypoints for the track. 
- The `waypoint_updater` node finds for the closest waypoint in front of the car, the the node publishes a list of the next 
`LOOKAHEAD_WPS=200` waypoints to the `/final_waypoints` topic (*x*, *y* coordinates, and target linear *velocity* of each waypoint).
- The `waypoint_follower` node subscribes to the `/final_waypoints` topic to determine the target linear and angular velocity for the car.
- The `waypoint_updater` node subscribes to the `/traffic_waypoint` topic that includes the nearest waypoint to the 
stop position (in front of a red light). The car will adjust its velocity to appropriate with the traffic light signals.

### 2. DBW_node (drive-by-wire node)
The code of this node is in [./ros/src/twist_controller/dbw_node.py](./ros/src/twist_controller/dbw_node.py).<br>
- The `dbw_node` node subscribes to the `/current_velocity` and `/twist_cmd` topics, and use various controllers 
to provide appropriate throttle, brake, and steering commands.

- The `dbw_node` node publishes these topics:
    - `vehicle/steering_cmd`
    - `vehicle/throttle_cmd`
    - `vehicle/brake_cmd`

### 3. Traffic Light Detector
The code of this part is in [./ros/src/tl_detector/tl_detector.py](./ros/src/tl_detector/tl_detector.py).
- The traffic light detection node `(tl_detector.py)` subscribes to 4 topics:
    - `/base_waypoints` provides the complete list of waypoints for the course.
    - `/current_pose` can be used to determine the vehicle's location.
    - `/image_color` which provides an image stream from the car's camera. These images are used to determine the color of 
    upcoming traffic lights.
    - `/vehicle/traffic_lights` provides the *(x, y, z)* coordinates of all traffic lights.
    
- The node publishes the index of the waypoint for nearest upcoming red light's stop line to a single topic: `/traffic_waypoint`

## Installation instructions
More information [https://github.com/udacity/CarND-Capstone](https://github.com/udacity/CarND-Capstone)

### 1. Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### 2. Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### 3. The pre-built image
Download the provided ubuntu image from Udacity [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/Udacity_VM_Base_V1.0.0.zip) 
to run the code, use a Ubuntu host machine to run the simulator.

### Config Network: Port Forwarding
Read instruction [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Port+Forwarding.pdf)

### Download the simulator
The simulator could be downloaded from [here](https://github.com/udacity/CarND-Capstone/releases)

## Usage

1. Clone the project repository
```bash
git clone https://github.com/maudzung/Self-Driving-Car-09-Programing-A-Real-Car.git
```

2. Install python dependencies
```bash
cd Self-Driving-Car-09-Programing-A-Real-Car
pip install -r requirements.txt
```
if having error with `scipy`, execute:
```bash
sudo apt-get install python-scipy
```

3. Make and run styx <br>
Add execute mode to *py
```bash
chmod -R +x ./ros/src
```

```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
```bash
wget https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip 
```
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd Self-Driving-Car-09-Programing-A-Real-Car/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images

### Other library/driver information
Outside of `requirements.txt`, here is information on other driver/library versions used in the simulator and Carla:

Specific to these libraries, the simulator grader and Carla use the following:

|        | Simulator | Carla  |
| :-----------: |:-------------:| :-----:|
| Nvidia driver | 384.130 | 384.130 |
| CUDA | 8.0.61 | 8.0.61 |
| cuDNN | 6.0.21 | 6.0.21 |
| TensorRT | N/A | N/A |
| OpenCV | 3.2.0-dev | 2.4.8 |
| OpenMP | N/A | N/A |






