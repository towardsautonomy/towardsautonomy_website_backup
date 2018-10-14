---
permalink: /robotics
---

---
# Robotics <a href="../../index.html"><img style="float: right;" src="/img/logo_circle.png" height="100" width="100">   

###### Author: *[Xianglong Lu](https://www.linkedin.com/in/xianglonglu/)*   

---

In recent years, with the improvement of economy and society, road capacity and traffic safety are becoming serious problems. Heavy driving work and fatigue driving are two key reasons causing traffic accidents. In this case, how to improve traffic safety has become a fatal social issue. These problems have motivated new researches and applications, for example, the self-driving vehicles, which can achieve better road capacity and safer driving by using control and SLAM algorithms, etc.

As the evolution of electromechanical and computing technologies continue to accelerate, the possible applications continue to grow. This accelerated growth is observed within the robotics research. New technologies (e.g. Arduino, Raspberry Pi with compatible interfaces, software and actuators/sensors) now permit young hobbyists and researchers to perform very complicated tasks - tasks that would have required great hardware/programming expertise just a few years ago. Within this article, current off-the-shelf technologies (e.g. Arduino, Raspberry Pi, commercially available chassis kit) are exploited to develop low-cost ground vehicles that can be used for multi-vehicle robotics research. Short-term, the goal is to develop several low cost ground vehicle platforms that can be used for multi-vehicle robotics research. This goal is intended as a first step toward the longer-term goal of achieving a fleet of Flexible Autonomous Machines operating in an uncertain Environment (FAME). Such a fleet can involve multiple ground and air vehicles that work collaboratively to accomplish coordinated tasks. Potential applications can include: Remote sensing, mapping, intelligence gathering, intelligence-surveillance-reconnaissance (ISR), search, rescue and much more. It is this vast application arena as well as the ongoing accelerating technological revolution that continues to fuel robotic vehicle research.

This article addresses modeling, design and control issues associated with ground based robotic vehicle. Particularly, LIDAR was used to implement Simultaneous Localization And Mapping (SLAM) algorithm (hector mapping) to perform indoor robot localization and mapping. Toward the longer-term FAME goal, several critical objectives are addressed. One central objective of the article was to show how to use low-cost chassis kit and convert it into somewhat “intelligent” multi-capability robotic platform that can be used for conducting FAME research. This article focuses on a rear-wheel drive robot (called FreeSLAM). Kinematic and dynamical models are examined. Rear-wheel drive means that the speed of the rear wheels are the same and controlled by a single dc motor (in our case two motors are treated identically and issued same voltage command). This vehicle class is non-holonomic: i.e. the two (2) (x, y) or (v, θ) controllable degrees of freedom is less than the three (3) total (x, y, θ) degrees of freedom. It is shown how continuous linear control theory can be used to develop suitable control laws that are essential for achieving various critical capabilities (e.g. speed control, control along a line/path, finish the track in minimum time, etc). Once the basic control issues are addressed, the vision-based lateral model is explained in detail. According to this model, three key parameters will greatly influence the tracking performance: robot cruise speed, fixed look-ahead distance and delay from vision subsystem. Each case above was well tested and discussed. Hector Mapping, which is one of the popular SLAM approaches to solve indoor SLAM

Contact [Xianglong Lu](https://www.linkedin.com/in/xianglonglu/) for any questions or concerns.

1. [What do we need to build a ground robot](/cv/semseg)
