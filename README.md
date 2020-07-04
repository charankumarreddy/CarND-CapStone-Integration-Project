# Project: Programming a Real Self-Driving Car

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).


## System Architecture Diagram
![System Architecture Diagram](/imgs/final-project-ros-graph-v2.png)

## Components
## Waypoint Updater
### Subscriber

1. /current_pose
2. /base_waypoints
3. /traffic_waypoints

### Publisher

1. /final_waypoints

![waypoint-updater-ros-graph](/imgs/waypoint-updater-ros-graph.png)

This module publishes a subset of /base_waypoints to /final_waypoints. The first waypoint in the list is the first waypoint that is currently ahead of the car. Also, we need to adjust the target velocities for the waypoints leading up to red traffic lights in order to bring the vehicle to a smooth and full stop.

The vehicle decelerates the speed according to the following formula:

    def decelerate_waypoints(self,waypoints,closest_idx):
        decelerated_waypoints = []

        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose

            stop_idx =   max(self.stopline_wp_idx - closest_idx - STOP_LINE_MARGIN, 0)
            dist = self.distance(waypoints, i, stop_idx)

            vel = math.sqrt(2 * MAX_DECEL * dist)

            if(vel < 1.0):
                vel = 0
            
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            decelerated_waypoints.append(p)

        return decelerated_waypoints



## Control
### Subscriber

1. /current_velocity
2. /twist_cmd
3. /vehicle/dbw_enabled

### Publisher

1. /vehicle/steering_cmd
2. /vehicle/throttle_cmd
3. /vehicle/brake_cmd

![dbw-node-ros-graph](/imgs/dbw-node-ros-graph.png)

This module publishes three commands throttle, brake and steering.

Throttle and brake values are calculated by the following steps:

1. To reduce noise, apply a low pass filter to the current velocity.
2. Calculate the difference between the current velocity and the target velocity, and calculate the throttle value with the PID controller.
3. If the target velocity and the current velocity are almost 0, set the throttle to zero and apply the meximum braking.
4. If the target velocity is lower than the current velocity, calculate the brake value by taking care of the vehicle mass, the wheel radius and the velocity error.

    **Steering value is calculated by the following steps:**

    1. To reduce noise, apply a low pass filter to the current velocity.
    2. Pass the linear velocity, the angular velocity and the current velocity to the yaw controller and calculate the throttle value.
    3. Apply low pass filter to the output steering value.
    4. For low pass filters and PID filter, We chose each parameter as follows.

    

### PID filter

Kp | Ki | Kd | min | max
-- | -- | -- | --- | ---
0.3 | 0.1 | 0. | 0. | 0.2

**Low pass filter (current velocity)**

tau | ts
--- | --
0.5 | 0.02

**Low pass filter (steering)**

tau | ts
--- | --
1 | 1

### Perception
The main task of the Perception module is detection and classification of obstacles in front of the vehicle. In case of this project, a front camera is used to analyze surrounding area of vehicle only and obstacles are traffic lights.

**The Perception module processes several input signals:**

1. /current_pose - position of the vehicle
2. /base_waypoints - list of waypoints
3. /image_color - image from the front camera
4. traffic_light_config.yaml - list of stop line positions before each traffic lights

![tl-detector-ros-graph](/imgs/tl-detector-ros-graph.png)

**Workflow of this module is following:**

1. Vehicle’s waypoint - Based on vehicle's position on the track [/current_pose], find the closest waypoint [/base_waypoints] with use of Euclidian distance.
2. Traffic light’s waypoint - Based on stop lines position in the map (located at sim_traffic_light_config.yaml), find the closest waypoint with use of Euclidian distance.
3. Detect closest Traffic light’s waypoint in front of the vehicle.
4. Measure distance between vehicle’s waypoint and the closest traffic light’s waypoint.
5. If the distance is smaller than some threshold, take an image from vehicles’s camera [/image_color] and continue with next steps. If there is no close traffic light start from step 1 again.
6. Run Traffic Light Detector and classifier for the camera’s image [from tl_classfier.py]
7. If red light was detected, send information about closest red-light traffic light waypoint to Waypoint Updater Node [/traffic_waypoint]


### Traffic lights detection and classification
Detection and classification of traffic lights on a captured image from front camera is done by Deep learning technique. Two algorithms and architectures were tested:

Single shot detector v2 (SSD v2) algorithm based on Inception architecture
Faster RCNN algorithm based on Resnet101 architecture

### Preparation
You have to download the following model files

for simulator [here](ros/src/tl_detector/light_classification/sim_traffic_img_classification.pb)

for realworld [here]([here](ros/src/tl_detector/light_classification/stest_site_traffic_img_classification.pb))

The following is a graph with the x axis as the distance to the stop line and the y axis as the speed.


## Simulater Video 
![simulation video](https://youtu.be/ojhbzBOSHbQ)

## Test site Video
![test site video](https://youtu.be/Jn7Pj6VcnQg)
Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

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

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
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
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
