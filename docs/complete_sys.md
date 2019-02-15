---
permalink: /complete_sys
---

# Complete Autonomous Cars / Robots In Action <a href="../../index.html"><img style="float: right;" src="/img/logo_circle.png" height="100" width="100">

###### Author: *[Shubham Shrivastava](http://www.towardsautonomy.com/#shubham)*   

---

Building a complete autonomous car or a robotic system requires implementing following key components: perception system for sensing the environment, path planning, trajectory generation and follower subsystem, and controls subsystems. Perception subsystem includes sensors like cameras, LiDARs, and RADARS, used to sense the environment, localize the vehicle/robot, and plan trajectories such that it avoids any collision while traveling to its destination. Perception is one of the biggest building block necessary for autonomous navigation. Path Planning module is responsible for planning the trajectory for the vehicle/robot to follow in order to reach its destination. This module takes into account the objects detected by perception system and plans to avoid either getting too close or hitting them. It utilizes a known map to understand the constraints such as one-way roads,and other non-drivable area to plan a global path to destination. Finally, controls module provides commands to actuate and steer the car/robot to follow the trajectory very closely and get to the destination. Usually two separate controllers are employed for longitudinal and lateral movements; one controller is used to control the throttle/brake while the other controls the steering of the vehicle. Sections below demonstrates implementation of such systems which incorporates some/all of  the key components necessary for autonomous navigation.

### TurtleBot3 Burger Autonomous Navigation in a Hallway

[![TurtleBot Autonomous Navigation](/docs/complete_sys/img/turtlebot_gazebo.png)](https://youtu.be/Pg-DAB2tA5c "TurtleBot Autonomous Navigation")

This is the simplest possible demonstration of an autonomous navigation system which implements Perception, Controls, and Path Planning. It demonstrates how these subsystems interacts with each other as a whole in order to sense the surroundings, plan its path, and get to its destination. The complete implementation is within the ROS framework.

TurtleBot3 Burger has found itself in a hallway. We know the walls do not go on forever, but we don’t know how long they extend. Each time we run the simulation, the walls might extend a different amount. The task is to get the burger to move until the end of the hallway, turn around and return to the original position.

The complete implementation details, resources, and source code can be found [here](https://github.com/towardsautonomy/towardsautonomy.github.io/tree/master/projects/turtlebot_nav)

### Self-Driving Car in Udacity Simulator

[![Udacity Self-Driving Car](/docs/complete_sys/img/self_driving_car_simulator.png)](https://youtu.be/aHQQqLsYMUg "Udacity Self-Driving Car")

This project demonstrates core functionality of the autonomous vehicle system, including traffic light detection, control, and waypoint following. The following is a system architecture diagram showing the ROS nodes and topics used in the project.

![](/docs/complete_sys/img/udacity_sim_project-rosgraph.png)

Here is the brief description of each major node doing the heavy lifting towards making the autonomous car run smoothly stopping at each red traffic light and completing the 5 mile loop.  

**Waypoint Updater Node (Part 1)**: This node subscribes to ```/base_waypoints``` and ```/current_pose``` and publishes to ```/final_waypoints```.  
**DBW Node**: Once your waypoint updater is publishing ```/final_waypoints```, the ```/waypoint_follower``` node will start publishing messages to the ```/twist_cmd``` topic.  
**Traffic Light Detection***: This is split into 2 parts:  
 * *Detection*: Detect the traffic light and its color from the ```/image_color```.
 * *Waypoint publishing*: Once the traffic lights are correctly identified the traffic light and determined its position, it can be converted to a waypoint index and published.     

**Waypoint Updater Node (Part 2)**: It uses ```/traffic_waypoint``` to change the waypoint target velocities before publishing to ```/final_waypoints```. The car, following this waypoint now stops at red traffic lights and move when they are green.

The complete implementation details, resources, and source code can be found [here](https://github.com/towardsautonomy/sandbox/tree/master/CarND-Capstone)
