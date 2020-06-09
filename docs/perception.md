---
permalink: /perception
---

<a href="../../index.html"><img style="float: left;" src="/img/back_button.png" height="25" width="25">

# Perception 

###### Author: *[Shubham Shrivastava](http://www.towardsautonomy.com/#shubham)*   
---

**Perception** is one of the most significant systems of autonomous vehicles/robots. It allows the vehicle to perceive the 360-degrees environment around it. Perception system helps the vehicle understands its surroundings like drivable area, lanes, vehicles, pedestrians, road boundaries, traffic signs, and traffic lights. This system uses multiple sensors like LiDAR, RADAR, and Camera to perceive the world and detect objects. This page explores various subsystems of perception and shows some methods of ground-plane segmentation, object detection, lane detection, and object classification.

---

### [CubifAE-3D: Monocular Camera Space Cubification on Autonomous Vehicles for Auto-Encoder based 3D Object Detection](https://www.towardsautonomy.com/CubifAE-3D/)

Authors: [Shubham Shrivastava](https://www.linkedin.com/in/shubshrivastava/) and [Punarjay Chakravarty](https://www.linkedin.com/in/punarjay-chakravarty/)

#### Abstract

We introduce a method for 3D object detection using a single monocular image. Depth data is used to pre-train an RGB-to-Depth Auto-Encoder (AE). The embedding learnt from this AE is then used to train a 3D Object Detector (3DOD) CNN which is used to regress the parameters of 3D object poses after the encoder from the AE generates a latent embedding from the RGB image. We show that we can pre-train the AE using paired RGB and depth images from simulation data once and subsequently only train the 3DOD network using real data, comprising of RGB images and 3D object pose labels (without the requirement of dense depth). Our 3DOD network utilizes a particular <i>cubification</i> of 3D space around the camera, where each cuboid is tasked with predicting N object poses, along with their class and confidence values. The AE pre-training and this method of dividing the 3D space around the camera into cuboids give our method its name - CubifAE-3D. We demonstrate results for monocular 3D object detection on the Virtual KITTI 2, KITTI, and nuScenes datasets for Autnomous Vehicle (AV) perception.  

![](/CubifAE-3D/resources/demo.gif)

---

### [DAC-DC : Divide and Concquer for Detection and Classification](https://github.com/towardsautonomy/DAC-DC)

![](/docs/perception/img/dac-dc.gif)

This is a modified version of YOLO for performing 2D object detection and tracking. The model was trained and tested on several datasets and seems to be performing quite well. The results shown here are for virtualKITTI dataset across multiple weather conditions and camera positions.

![](/docs/perception/img/dacdc-result.png)
*Figure 1: Result of DAC-DC on virtualKITTI dataset. Blue boxes are ground-truth, and red boxes are predictions*

#### [GitHub](https://github.com/towardsautonomy/DAC-DC)

---

### [LiDAR based Ground-Plane Segmentation and Object Detection](https://github.com/towardsautonomy/TAPL#lidar-object-detection)  
![](/docs/perception/img/lidar_object_detection.gif)

#### This has been implemented as part of [TAPL](https://www.towardsautonomy.com/tapl/index.html). The pipeline can be explored [here](https://github.com/towardsautonomy/TAPL#lidar-object-detection)

**Point-Cloud** is a set of data points in 3D space which represents the LiDAR laser rays reflected by objects. Each point within the point-cloud is the point of interaction between the transmitted ray and the environment. Ground plane can be easily segmented out by finding planes within the point-cloud and selecting the one with maximum number of inliers. This can be achieved using a very well known algorithm, RANSAC (RANdom SAmple Consensus).

**RANSAC** first picks a few samples randomly within the point-cloud and fits a plane through that. Then it counts the number of inliers by computing closest distance between the plane and all other points. If this distance is within a certain threshold, then this point is added to the list of inliers. This process is repeated multiple number of times to find the fit that estimates the ground-plane. A plane can be fit through points using either SVD or Least-Squares method.

Once we have segmented the ground-plane, we can look into all other points to find objects (if any) within the point-cloud. To find an object, we can take advantage of the fact that the points corresponding to the object are distributed very close to each other. If we can find clusters of points within the point-cloud, then the clusters will correspond to objects like vehicles, pedestrians, road boundary walls, buildings, and trees. Further, we can employ point-cloud filtering using multiple constraints such as *bounding-box* length, width, and height to differentiate between different classes. A 3D **bounding-box** defines the boundary of objects such as *x_min, x_max, y_min, y_max, z_min, and z_max*.

**Clustering** requires looking for all the points closer to a *seed* point. The computation can very quickly increase exponentially in the brute-force method of computing distance between all the possible points. If the points are stored in a k-d tree structure, then the search problem becomes much easier and computationally cheap. **Euclidian Clustering** method is utilized in this project to find clusters within the point-cloud. This clustering method uses k-d tree search to find points that are close together.

![](/docs/perception/img/kd_tree.png)  
*Figure: k-d tree visualization for in 3D space*

k-d tree insert and search methods, euclidean clustering, line and plane fitting, and RANSAC are implemented as part of TAPL.

---

#### [Semantic Segmentation](https://github.com/towardsautonomy/ssnet_semseg)
![](/docs/perception/img/semseg.gif)

[![Semantic Segmentation](/docs/dl/img/semseg/thumbnail.png)](https://youtu.be/HzW1ZUwmlTQ "Semantic Segmentation")

Semantic Segmentation is a fascinating application of deep learning and has become very popular among machine learning researchers. Semantic Segmentation or more commonly known as SemSeg is understanding an image at pixel level. Technically speaking, it is a CNN (Convolution Neural Network) which can classify every single pixel in an image as an object class. This also paves the way towards complete scene understanding. So much research have been done in this area since 2014 (Benchmarking Data: [https://www.cityscapes-dataset.com/benchmarks/](https://www.cityscapes-dataset.com/benchmarks/)) and now we are at a point where we have enough data and computational power to actually start seeing SemSeg in our life. SemSeg can also be applied to videos and 3D point-cloud data to obtain fine-grained semantics. Some of the commonly known CNN architectures for SemSeg are: FCN, VGG-16, SegNet, DeepLap, Dilated Convolutions

Inferring the knowledge about an image has a number of applications including in autonomous driving. SemSeg can help autonomous vehicles learn about its surroundings, specifically inferring the information about free road space and objects around it. Successful and accurate pixel-level prediction however depends on how well the network has been trained. Thanks to organizations like Cityscapes and KITTI, today we have large fine-annotated pixel-level datasets available to work with.

Here I am presenting a 20 layer CNN architecture I created: SSNet. This architecture is inspired by VGG-16 and SegNet and is shown below. This is an encoder-decoder architecture in which we have 10 encoder layers followed by 10 corresponding decoder layers. Number of filters for each encoder layer are: first 2 layers - 64 filters, next 2 layers - 128 filters, next 3 layers - 256 filters, next 3 layers - 512 filters. The corresponding decoder have same number of filters. The decoder layers are followed by a softmax prediction layer with number of filters equal to the number of prediction classes.

![](/docs/dl/img/semseg/SSNet.png)

This network was trained on just 200 labelled training images obtained from [KITTI pixel-level semantic dataset](http://www.cvlibs.net/datasets/kitti/eval_semseg.php?benchmark=semantics2015). The results shown in this post were generated by running this network on KITTI testing datasets. The video shown at the top only displays road and vehicle classes. Few semantic results generated for all the classes are shown below.  

| Test Image                        |  SSNet Prediction                 |
|:---------------------------------:|:---------------------------------:|
|![](/docs/dl/img/semseg/test1.png) | ![](/docs/dl/img/semseg/pred1.png)|
|![](/docs/dl/img/semseg/test2.png) | ![](/docs/dl/img/semseg/pred2.png)|
|![](/docs/dl/img/semseg/test3.png) | ![](/docs/dl/img/semseg/pred3.png)|

#### [GitHub](https://github.com/towardsautonomy/ssnet_semseg) 

Pretrained weights can be downloaded [here](https://drive.google.com/open?id=1KG_-paGZmyxnSfPZGEv7uq_vTduXrLr3).

---

#### [Lane Detection and Tracking using Birds-Eye view](/perception/lane_detection)  
![](/docs/perception/img/lane_detection/straight_lines1.jpg)

---

#### [Camera Based Image Feature Detection and Tracking](https://github.com/towardsautonomy/TAPL#image-feature-detection-and-tracking)
![](/docs/perception/img/matching_points.png)

#### This has been implemented as part of [TAPL](https://www.towardsautonomy.com/tapl/index.html). The pipeline can be explored [here](https://github.com/towardsautonomy/TAPL#image-feature-detection-and-tracking)

---