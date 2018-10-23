---
permalink: /dl/obj_detection
---

## Object Detection, Classification, and Localization <a href="../../index.html"><img style="float: right;" src="/img/logo_circle.png" height="100" width="100">

###### Author: *[Shubham Shrivastava](http://www.towardsautonomy.com/#shubham)*   

---

[![Object Detection](/docs/dl/img/obj_detection/thumbnail.png)](https://youtu.be/GwxhcyTnxoE "Object Detection")

The importance of Object Detection in autonomous driving is a no brainer. For autonomous driving cars it is imperative that it knows about the objects around it. Recognizing objects is of equal importance, reason being that different objects behave differently. Pedestrians movement for example could be  completely arbitrary as opposed to a car which can only move front and back in small curves, but never sideways. At a minimum, detecting vehicles, pedestrians, traffic signs, and traffic lights is crucial for self-driving cars. 

Finding object distance from the car with a high degree of confidence is not possible using just a mono camera. However, Stereo Camera pair or sensors like LIDAR or RADAR could be used to find distance from these objects with high confidence. Sensor Fusion can then be used to combine these information and provide these to the systems responsible for taking driving actions for the car.

A lot of research have been in going on in this area and the most common approach for object detection is by using CNNs. Some of the well known CNN models for object detection and recognition are OverFeat, VGG16, Fast R-CNN, YOLO, and SimpleNet [[2]](#references). YOLO (You Only Look Once) [[1]](#references) is of a particular interest to me, it looks at object detection as a regression problem to spatially separated bounding boxes and associated class probabilities. YOLO can simultaneously predict multiple bounding boxes and class probabilities for those boxes. This network trains on full images and directly optimizes detection performance. YOLO doesn't need an extremely complex pipeline since it frames the detection as a regression problem, and for that reason it is extremely fast.

I used YOLOv2 for object detection and localization. Each image is first resized to 608x608 and then divided into a 19x19 grid. The network then predicts label probabilities and bounding-box for each grid. Each grid can predict more than one class and the number of possible objects depends on the number of anchors used. For one anchor the output label for each grid looks like this: Y = [Pc, Bx, By, Bh, Bw, Pc1, Pc2, ... , Pcn]; where Pc is the probability that an object exists in the grid cell; Bx, By, Bh, Bw corresponds to the bounding-box of the detected object; Pc1, Pc2, ... , Pcn provides the probability of objects being of a certain class, n being the number of classes. It is obvious that if Pc is below certain threshold (example: 0.6) that means no object exists within that grid cell and all other label parameters do not matter. Each grid can contain multiple objects which means that we should output multiple labels. Defining 5 anchors means that each grid can predict a maximum of 5 objects contained within the cell. So, the output label dimensions for 19x19 grids, 5 anchors, and 80 classes would be 19x19x5x85.

Another problem with this kind of detection is multiple overlapping bounding-boxes. This is due to the fact that multiple grid cells might think that it contains the center of object. But this could be solved by using Intersection over Union (IoU) and applying Non-Max Suppression. To eliminate overlapping bounding-boxes we first find the highest confidence bounding-box and then compute intersecting over union for all other bounding-boxes. High IoU means there is a high overlap and so we can those bounding-boxes. Then we start with next highest confidence bounding-box and repeat the process.

You can find my code and instructions [here](https://github.com/towardsautonomy/towardsautonomy.github.io/tree/master/projects/object_detection_yolov2)

Pretrained weights can be found [here](https://drive.google.com/open?id=1akSwgUkqavf7upvB9WigNLhkQIFt_3je)  

For more information, contact [Shubham Shrivastava](http://www.towardsautonomy.com/#shubham)

#### References

[1] https://arxiv.org/abs/1506.02640  
[2] https://web.stanford.edu/class/cs231a/prev_projects_2016/object-detection-autonomous.pdf