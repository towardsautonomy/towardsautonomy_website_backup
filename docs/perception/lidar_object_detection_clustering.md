---
permalink: /perception/lidar_object_detection_clustering
---

## LiDAR based Ground-Plane Segmentation and Object Detection <a href="../../index.html"><img style="float: right;" src="/img/logo_circle.png" height="100" width="100">

###### Author: *[Shubham Shrivastava](http://www.towardsautonomy.com/#shubham)*   

---

![](/docs/perception/img/lidar_objects_front_side.gif)

## Introduction

**Lidar** sensing gives us high resolution data by sending out thousands of laser signals. These lasers bounce off objects, returning to the sensor where we can then determine how far away objects are by timing how long it takes for the signal to return. Also we can tell a little bit about the object that was hit by measuring the intesity of the returned signal. Each laser ray is in the infrared spectrum, and is sent out at many different angles, usually in a 360 degree range. While lidar sensors gives us very high accurate models for the world around us in 3D.

## How to find objects withing point-cloud

**Point-Cloud** is a set of data points in 3D space which represents the LiDAR laser rays reflected by objects. Each point within the point-cloud is the point of interaction between the transmitted ray and the environment. Ground plane can be easily segmented out by finding planes within the point-cloud and selecting the one with maximum number of inliers. This can be achieved using a very well known algorithm, RANSAC (RANdom SAmple Consensus).

**RANSAC** first picks a few samples randomly within the point-cloud and fits a plane through that. Then it counts the number of inliers by computing closest distance between the plane and all other points. If this distance is within a certain threshold, then this point is added to the list of inliers. This process is repeated multiple number of times to find the fit that estimates the ground-plane. A plane can be fit through points using either SVD or Least-Squares method.

Once we have segmented the ground-plane, we can look into all other points to find objects (if any) within the point-cloud. To find an object, we can take advantage of the fact that the points corresponding to the object are distributed very close to each other. If we can find clusters of points within the point-cloud, then the clusters will correspond to objects like vehicles, pedestrians, road boundary walls, buildings, and trees. Further, we can employ point-cloud filtering using multiple constraints such as *bounding-box* length, width, and height to differentiate between different classes. A 3D **bounding-box** defines the boundary of objects such as *x_min, x_max, y_min, y_max, z_min, and z_max*.

**Clustering** requires looking for all the points closer to a *seed* point. The computation can very quickly increase exponentially in the brute-force method of computing distance between all the possible points. If the points are stored in a k-d tree structure, then the search problem becomes much easier and computationally cheap. **Euclidian Clustering** method is utilized in this project to find clusters within the point-cloud. This clustering method uses k-d tree search to find points that are close together.

![](/docs/perception/img/kd_tree.png)  
*k-d tree visualization for in 3D space*

k-d tree insert and search methods, euclidean clustering, line and plane fitting, and RANSAC are implemented as part of TAPL.

## This has been implemented as part of [TAPL](https://www.towardsautonomy.com/tapl/index.html). The pipeline can be explored [here](https://github.com/towardsautonomy/TAPL#lidar-object-detection)