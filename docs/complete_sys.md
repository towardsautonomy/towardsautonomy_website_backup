---
permalink: /complete_sys
---

# Complete Autonomous Cars / Robots In Action <a href="../../index.html"><img style="float: right;" src="/img/logo_circle.png" height="100" width="100">

###### Author: *[Shubham Shrivastava](http://www.towardsautonomy.com/#shubham)*   

---

Building a complete autonomous car or a robotic system requires implementing following key components: perception system for sensing the environment, path planning, trajectory generation and follower subsystem, and controls subsystems. Perception subsystem includes sensors like cameras, LiDARs, and RADARS, used to sense the environment, localize the vehicle/robot, and plan trajectories such that it avoids any collision while traveling to its destination. Perception is one of the biggest building block necessary for autonomous navigation. Path Planning module is responsible for planning the trajectory for the vehicle/robot to follow in order to reach its destination. This module takes into account the objects detected by perception system and plans to avoid either getting too close or hitting them. It utilizes a known map to understand the constraints such as one-way roads,and other non-drivable area to plan a global path to destination. Finally, controls module provides commands to actuate and steer the car/robot to follow the trajectory very closely and get to the destination. Usually two separate controllers are employed for longitudinal and lateral movements; one controller is used to control the throttle/brake while the other controls the steering of the vehicle. Sections below demonstrates implementation of such systems which incorporates some/all of  the key components necessary for autonomous navigation.

---

### TurtleBot3 Burger Autonomous Navigation in a Hallway

[![TurtleBot Autonomous Navigation](/docs/complete_sys/img/turtlebot_gazebo.png)](https://youtu.be/Pg-DAB2tA5c "TurtleBot Autonomous Navigation")

This is the simplest possible demonstration of an autonomous navigation system which implements Perception, Controls, and Path Planning. It demonstrates how these subsystems interacts with each other as a whole in order to sense the surroundings, plan its path, and get to its destination. The complete implementation is within the ROS framework.

TurtleBot3 Burger has found itself in a hallway. We know the walls do not go on forever, but we don’t know how long they extend. Each time we run the simulation, the walls might extend a different amount. The task is to get the burger to move until the end of the hallway, turn around and return to the original position.

##### The complete implementation details, resources, and source code can be found [here](https://github.com/towardsautonomy/towardsautonomy.github.io/tree/master/projects/turtlebot_nav)

---

### Self-Driving Car in Udacity Simulator

##### [YouTube Link to the demo video](https://youtu.be/aHQQqLsYMUg "Udacity Self-Driving Car")

This project demonstrates core functionality of the autonomous vehicle system, including traffic light detection, control, and waypoint following.

![](/docs/complete_sys/self_driving_car_simulator.png)

The following is a system architecture diagram showing the ROS nodes and topics used in the project.

![](/docs/complete_sys/img/udacity_sim_project-rosgraph.png)

Here is the brief description of each major node doing the heavy lifting towards making the autonomous car run smoothly stopping at each red traffic light and completing the 5 mile loop.  

**Waypoint Updater Node (Part 1)**: This node subscribes to ```/base_waypoints``` and ```/current_pose``` and publishes to ```/final_waypoints```.  
**Waypoint Updater Node (Part 1)**: This node subscribes to ```/base_waypoints``` and ```/current_pose``` and publishes to ```/final_waypoints```.  
**DBW Node**: Once the waypoint updater is publishing ```/final_waypoints```, the ```/waypoint_follower``` node will start publishing messages to the ```/twist_cmd``` topic.  
**Traffic Light Detection**: This is split into 2 parts:  
 * *Detection*: Detect the traffic light and its color from ```/image_color```.
 * *Waypoint publishing*: Once the traffic lights are correctly identified the traffic light and determined its position, it can be converted to a waypoint index and published.     

**Waypoint Updater Node (Part 2)**: It uses ```/traffic_waypoint``` to change the waypoint target velocities before publishing to ```/final_waypoints```. The car, following this waypoint now stops at red traffic lights and move when they are green.   
**Twist Controller**: It executes the twist commands published by ```/waypoint_follower``` node over ```/twist_cmd``` topic.

---

### Waypoint Updater Node

 * This node publishes final waypoints on the topic ```/final_waypoints``` at the rate 50Hz   
 * The callback function for topic ```/base_waypoints``` stores base waypoints into a global variable to use throughout the program.   
 * The callback function for topic ```/current_pose``` updates the vehicle pose every time a new message is received.   
 * The function ```closest_waypoint()``` returns the index of closest waypoint ahead of current pose. It first computes the index of closest waypoint to the current pose and then finds whether of not it is ahead of current pose by computing the dot product between two vectors: (1) current pose to closest waypoint vector (2) previous waypoint to the closest waypoint vector. If the dot product is positive, that means that both vectors are in the same direction meaning closest waypoint found is ahead of the vehicle which index is then returned. If the dot product result is negative, that means that they are in opposite direction meaning closest waypoint found is behind the vehicle and next index (index+1) is returned.   
 * This node also subscribes to the ```/traffic_waypoint``` topic which is published by the ```/tl_detector``` node which is responsible for detecting and classifying the traffic lights ahead and publish the waypoint closest to a red light stop line.
 * If a red light was detected then this node updates the final waypoints so that the vehicle stops at the stop line.   
 * Final waypoints are updated to stop at red lights by the function ```update_waypoints()```. This function updates the waypoints by first finding a velocity decrease slope between ```STOP_DECEL_BEGIN_DIST``` meters from the traffic light stop line and ```STOP_LINE_ADVANCE_DIST``` meters from traffic light stop line which happens to be 40 meters and 7 meters in the code.   

### Traffic Light Detection

 * The node ```/tl_detector``` reads in traffic light positions from the config file *sim_traffic_light_config.yaml* and subscribes to ```/current_pose``` topic to determine if there are any upcoming traffic lights within ```RELEVANT_TRAFFIC_LIGHT_DIST``` meters ahead of the vehicle.   
 * Function ```pose_cb()``` is registered as the callback function for topic ```/current_pose```. Every time this function is called, it iterates through the list of traffic light stop line positions and computes the distance between the closest waypoint to the stop line and the closest waypoint to the vehicle. If this distance is withing ```RELEVANT_TRAFFIC_LIGHT_DIST``` meters and the stop line is ahead of the vehicle, then it sets ```upcoming_traffic_light``` to the coordinate belonging to that traffic light in the list obtained from the config file. ```upcoming_traffic_light``` is set to ```None``` otherwise.   
 * This node also subscribes to the camera frames coming over the topic ```/image_color``` and the registered callback function is ```image_cb()```.   
 * This ```image_cb()``` callback function first looks at ```upcoming_traffic_light``` to see if it is set to have the xy-coordinates of the traffic light. If it contains the xy-coordinates, then it calls the traffic light classifier to predict the states of traffic light.   
 * If a red traffic light was detected, then state debouncing is performed to make sure that the same state has been predicted at least ```STATE_COUNT_THRESHOLD``` number of times and then the waypoint index belonging to the closest waypoint from traffic light stop line is published over the topic ```/traffic_waypoint```.
 * If a red traffic light was not detected, then this node publishes ```-1``` over the topic ```/traffic_waypoint```.

### Traffic Light Classification

The following approach were taken for Traffic Light Classification:  
1. Download and install [Darknet](https://pjreddie.com/darknet/yolo/).   
2. Download pre-trained YOLOv2 tiny model [weights](https://pjreddie.com/media/files/yolov2-tiny.weights) and [config](https://github.com/pjreddie/darknet/blob/master/cfg/yolov2-tiny.cfg). (Since YOLOv3 is not compatible with OpenCV 3.3.1 used in ROS Kinetic, we chose to use YOLOv2.)   
3. YOLO detects 80 classes by default. Follow instructions [here](https://github.com/AlexeyAB/darknet/tree/47c7af1cea5bbdedf1184963355e6418cb8b1b4f#how-to-train-tiny-yolo-to-detect-your-custom-objects) to edit config file, create classes file (traffic_lights-obj.names), metadata file (traffic_lights.data) to adapt them to our need of detecting 3 classes (red, yello, green lights).   
4. Collect training image data from simulator and actual test track by installing and using ROS package `image_view`, as follows:   

`apt-get install -y ros-kinetic-image-view`

`rosrun image_view image_saver image:=/image_color`

and

`rosbag play -l ../data/traffic_light_training.bag`

`rosrun image_view image_saver image:=/image_raw _filename_format:="tlt%04i.%s" _sec_per_frame:=0.2`

5. Select training images - roughly equal number of red, yellow, green, and unknown/no light scenarios, and label them using a labeling tool. We used [labelImg](https://github.com/tzutalin/labelImg).   
6. Follow the instructions [here](https://github.com/AlexeyAB/darknet/tree/47c7af1cea5bbdedf1184963355e6418cb8b1b4f#how-to-train-tiny-yolo-to-detect-your-custom-objects) to start training a classifier for traffic light detection.      
7. Stop training when the average loss (error) is less than 0.5.   
8. Update tl_classifier.py to use the weights along with OpenCV API to predict traffic lights with states.. Ensure appropriate weights file is used for simulator vs. site case.   

### Twist Controller

 * Once the waypoint updater is publishing ```/final_waypoints```, the ```/waypoint_follower``` node will start publishing messages to the ```/twist_cmd``` topic.  
 * The file ```twist_controller/twist_controller.py``` defines and executes all the control functions to execute twist commands received over ```/twist_cmd``` topic.  
 * It uses two separate controllers for throttle/brake controls and steer controls.   
 * Throttle/Brake controller utilizes a simple PID controller defined in ```pid.py``` whereas Steer controller utilizes the yaw controller defined in ```yaw_controller.py```.    
 * Output of the controllers are smoothed out using a lowpass filter defined in ```lowpass.py``` before being published.   
 * The controller functions are executed only if *drive-by-wire (dbw)* is enabled.
 * Throttle value ranges from 0 to 1.0 however the braking torque ranges from 0 to 700 Nm. To get these two range of values, following approach was taken: (1) The range for PID controller is defined to be -1.0 to 1.0. (2) If the controller returns a value >= 0, the same value is assigned to throttle value. (3) If the controller returns a value < 0, that means that braking is required. Braking torque is then computed as a scaled value that is negatively proportional to the value returned by controller. i.e. ```brake = -1.0*controller_output*BRAKE_SLOWDOWN_TORQUE```.   
 * If the desired linear velocity is very small (< 0.1), that means the vehicle needs to come to a complete stop. Since the vehicle is an automatic-transmission, maximum braking torque is applied to make sure that the vehicle is not moving.   

##### The complete implementation details, resources, and source code can be found [here](https://github.com/towardsautonomy/towardsautonomy.github.io/tree/master/projects/self_driving_car_udacity)

---
