---
permalink: /sensor_fusion
---

# Sensor Fusion <a href="../../index.html"><img style="float: right;" src="/img/logo_circle.png" height="100" width="100">

###### Author: *[Shubham Shrivastava](http://www.towardsautonomy.com/#shubham)*   

---

Sensor Fusion is the process of intelligently combining data from several sensors for the purpose of improving localization performance and to increase our confidence in either our position or perceived/detected position of other vehicles. Data source for this fusion does not have to be identical. One could choose different sensors capable of detecting different aspect of the object better and later combine them all to get an accurate estimate of the object’s position. As an example, LIDARs can detect the position of objects accurately whereas RADARs can detect their velocity accurately. By combining these two sensors we can estimate the object’s State very precisely. increase our confidence in either our position or perceived/detected posiiton of other vehicles. Data source for this fusion does not have to be identical. One could choose different sensors capable of detecting different aspect of the object better and later combine them all to get an accurate estimate of the object's position. As an example, LIDARs can detect the position of objects accurately whereas RADARs can detect their velocity accurately. By combining these two sensors we can estimate the object's State very precisely.

Most commonly used sensors used for estimating host vehicle's position and localize itself are Global Positioning System (GPS) and Inertial Measurement Unit (IMU) wherein the GPS provides position information and the IMU provides information about its linear and angular movement. IMU's typically uses a combination of accelerometers, gyroscopes, sometimes also magnetometers and reports body's specific force, angular rate, and sometimes the magnetic field surrounding the body.

Many object detection methods have been in research in automotive industry for quite some time. These methods uses one of many sensors like Cameras, LIDARs, RADARs, V2X, Stereo Cameras. All these methods work on detecting objects and finding their current position and/or estimate their future position + trajectories. If the vehicle has more than one sensor installed, it could exploit these sensors by fusing their individual object detection result and obtain an accurate position that can be trusted with more confidence.

## [Kalman Filter](https://github.com/towardsautonomy/kalman_filter)

The Kalman Filter has long been regarded as the optimal solution to many tracking and data prediction tasks. It is composed of two basic functions: **Predict** and **Update**. The *Predict* function predicts vehicle state at timestamp *(k + 1)* given its state at timestamp *k* based on the knowledge of vehicle's dynamics which we represent as motion model. The *Update* model then updates/corrects the believe about its position at timestamp *(k + 1)* after a new measurement have been received. The predict and update equations provide a recursive way to compute the posterior of the state for every measurement that we receive. 

---
| Error in x position and velocity  |  Error in y position and velocity |
|:---------------------------------:|:---------------------------------:|
|![](/docs/sensor_fusion/x_err.png)             | ![](/docs/sensor_fusion/y_err.png)            |  

*Figure 1: X and Y position error plots*

---

![](/docs/sensor_fusion/states.png)  
*Figure 2: Plot of 2D position and velocity*

---

### [GitHub](https://github.com/towardsautonomy/kalman_filter) 

Want to learn more about Kamlan Filter? A good tutorial on Kalman Filter from MIT can be found [here](http://web.mit.edu/kirtley/kirtley/binlustuff/literature/control/Kalman%20filter.pdf). C++ and Python implementation for 1D and 2D Kalman Filter can be downloaded below.

Going little bit into the technicality here, Kalman Filter basically assumes the state and noise to be Gaussian and can be completely characterized by N(μ, σ²). Assuming the parameters to be Gaussian is the reason Kalman Filter works the best when measurement to estimation transformation is linear. For example, an object detection technique that uses a sensor which could provide objects x and y position directly would lead to a linear transformation for the Kalman Filter's 'Update' process. A non-linear transformation of a gaussian parameter results in a non-gaussian parameter and therefore Kalman Filter cannot be used in such cases for position estimation. However, this does not mean that we cannot deal with non-linear transformations. There is a non-linear extension of Kalman Filter called **Extended Kalman Filter** ([Find an interactive tutorial here](https://home.wlu.edu/~levys/kalman_tutorial/)) which linearizes the parameters about an estimate of the current mean and covariance. While EKF (Extended Kalman Filter) works well with non-linear transformation, it does have some weaknesses. For highly non-linear problems, the EKF encounters issues with both accuracy and stability. Also, Jacobian computations are required in order to linearize the non-linear function using a first-order Taylor series.

Truth be told, computing Jacobians are a pain. Plus the fact that EKF linearizes a non-linear function does not provide a very accurate result. Julier and Uhlmann proposed an alternative to the EKF called **[Unscented Kalman Filter](https://people.eecs.berkeley.edu/~pabbeel/cs287-fa15/optreadings/JulierUhlmann-UKF.pdf)** which works on the principal of unscented transformation. For unscented transformation we choose so-called Sigma Points using the mean, covariance and spreading parameters; and then we propagate each of these sigma points through the non-linearity yielding a set of transformed sigma points. The new estimated mean and covariance are then computed based on their statistics. Unscented Transformation is accurate to at least third-order for Gaussian systems.

Enough about filters now, let's see how to use them. Some of the most commonly used sensors for object detections are LiDAR, RADAR, and Camera. While LiDAR and Camera maps the world in cartesian coordinates, RADAR maps the world in polar coordinate. This means that going from the estimated state *{X = [x, y, Vx, Vy]}* to the measured state *{X = [rho, phi, rho_dot]}* involves use of an inverse-tangent which introduces a non-linearity.

So what needs to be done for fusing the LiDAR and RADAR based object detection results? The major difference is in the Kalman Filter update functions. After receiving the LiDAR data, update/correction can be performed using traditional Kalman Filter; however, when RADAR data is received, update/correction should be performed using either EKF or UKF.

An implementation of Sensor Fusion for RADAR and LiDAR data using Extended Kalman Filter can be found [here](https://github.com/towardsautonomy/towardsautonomy.github.io/tree/master/projects/sensor_fusion_ekf). The result of sensor fusion is shown below.

[![](/sensor_fusion/ekf_thumbnail.png)](https://youtu.be/283eQyVcZBc "Sensor Fusion - EKF")

An implementation of Sensor Fusion for RADAR and LiDAR data using Unscented Kalman Filter can be found [here](https://github.com/towardsautonomy/towardsautonomy.github.io/tree/master/projects/sensor_fusion_ukf). The result of sensor fusion is shown below.

[![](/sensor_fusion/ukf_thumbnail.png)](https://youtu.be/6sqc7zZwqv4 "Sensor Fusion - UKF")

In the videos above, Red markers are LIDAR measurements, Blue markers are RADAR measurements, and the Green markers is the position provided by Kalman Filter. It can be clearly seen that the Kalman Filter does a great job at estimating the Car's actual position. The performance of filter is measured using Root-Mean-Squared Error (RMSE) and is also shown in the videos. Also, note that the RMSE for UKF is smaller than the RMSE for EKF especially for Vx and Vy. It is due to the fact that RADARs can measure velocity directly unlike the LIDARs which infers objects velocity from its position.