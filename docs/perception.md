---
permalink: /perception
---

---
# Perception <a href="../../index.html"><img style="float: right;" src="/img/logo_circle.png" height="100" width="100">   

###### Author: *[Shubham Shrivastava](http://www.towardsautonomy.com/#shubham)*   
---

**Perception** is one of the most significant systems of autonomous vehicles/robots. It allows the vehicle to perceive the 360-degrees environment around it. Perception system helps the vehicle understands its surroundings like drivable area, lanes, vehicles, pedestrians, road boundaries, traffic signs, and traffic lights. This system uses multiple sensors like LiDAR, RADAR, and Camera to perceive the world and detect objects. This page explores various subsystems of perception and shows some methods of ground-plane segmentation, object detection, lane detection, and object classification.

### [LiDAR based Ground-Plane Segmentation and Object Detection](/perception/lidar_object_detection_clustering)  
![](docs/perception/img/lidar_objects_front_view.gif)

---

## Computer Vision

Computer vision is a field of computer science that works on enabling computers to see, identify and process images in the same way that human vision does, and then provide appropriate output. It is like imparting human intelligence and instincts to a computer. In reality though, it is a difficult task to enable computers to recognize images of different objects.

Computer vision is closely linked with machine learning and artificial intelligence, as the computer must interpret what it sees, and then perform appropriate analysis or act accordingly.

Computer vision's goal is not only to see, but also process and provide useful results based on the observation. For example, a computer could create a 3-D image from a 2-D image, such as those in cars, and provide important data to the car and/or driver. For example, cars could be fitted with computer vision which would be able to identify and distinguish objects on and around the road such as traffic lights, pedestrians, traffic signs and so on, and act accordingly. The intelligent device could provide inputs to the driver or even make the car stop if there is a sudden obstacle on the road.

When a human who is driving a car sees someone suddenly move into the path of the car, the driver must react instantly. In a split second, human vision has completed a complex task, that of identifying the object, processing data and deciding what to do. Computer vision's aim is to enable computers to perform the same kind of tasks as humans with the same efficiency.

Computer Vision can help Autonomous Cars perceive its surroundings and localize itself. It can do so by detecting the lanes, vehicles, pedestrians, traffic signs, and traffic lights on the road. Some of these are explained here and their implementation with results are given in their respective sections. Contact [Shubham Shrivastava](http://www.towardsautonomy.com/#shubham) for any questions or concerns.

---

#### [Lane Detection and Tracking using Birds-Eye view](/perception/lane_detection)  
![](docs/perception/img/lane_detection/straight_lines1.jpg)

---

#### [Object Detection, Classification, and Localization](/dl/obj_detection)
![](docs/dl/img/yolo_background.png)

---

#### [Vehicle Detection using SVM classifier](/perception/vehicle_detection)  
![](img/lane_veh.gif)

---

#### [Traffic Sign Classification using CNN](/perception/traffic_sign_classification)
![](docs/perception/img/traffic_sign_classification/test_detection.png)

---

#### [Semantic Segmentation](/dl/semseg)
![](docs/dl/img/semseg_cover.png)

---
