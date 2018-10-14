---
permalink: /v2x
---

# V2X (Vehicle-To-Everything) <a href="../../index.html"><img style="float: right;" src="/img/logo_circle.png" height="100" width="100">

###### Author: *[Shubham Shrivastava](http://www.towardsautonomy.com/#shubham)*  
**Cite this as:**  Shrivastava, S., *V2X (Vehicle-To-Everything)*, Retrieved from [http://www.towardsautonomy.com/v2x](http://www.towardsautonomy.com/v2x)  

---

[![V2X Demo](/docs/v2x/img/v2x-hmi.png)](https://youtu.be/DVGtUJRWjiA "V2X Demo")

National Highway Traffic Safety Administration (NTHSA) has been interested in vehicle to vehicle (V2V) communication as the next step in addressing grooving rates of fatalities from vehicle related crashes. Today’s crash avoidance technologies depend on on-board sensors like camera and radar to provide awareness input to the safety applications. These applications are warning the driver of imminent danger or sometimes even act on the driver's behalf. However, even technologies like those cannot "predict" a crash that might happen because of a vehicle which is not very close or not in the line of sight to the host vehicle. A technology that can "see" through another vehicle or obstacles like buildings and predict a danger can fill these gaps and reduce crashes drastically.V2V communications can provide vehicles the ability to talk to each other and therefore see around corners and through the obstacles over a longer distance compared to the current on-board sensors. It is estimated that V2X communications address up to 80% of the unimpaired crashes [2]. By means of Notice of Proposed Rulemaking(NPRM),NHTSA is working towards standardization of V2V communications and potentially mandating the broadcast of vehicle data (e.g. GPS coordinates, speed, acceleration) over DSRC through V2V.

A vehicle needs an On-Board Unit(OBU) to establish the V2V communication with other vehicles also equipped with OBUs or V2I communication with the traffic infrastructure equipped with Road-Side Units (RSUs).In general, an OBU has a DSRC radio for transmission and reception, GNSS receiver, a processor, and several interfaces (e.g. CAN, Ethernet, GPS) for obtaining the vehicle data. Essential message in V2V communication is called Basic Safety Messages (BSM). BSM is a broadcast message typically transmitted frequently up to 10 times a second. Content of BSM includes vehicle information such as vehicle speed, location, and brake status.

Safety applications use the remote vehicles (RVs) data from BSM and Host Vehicle (HV) data from the OBU interfaces like CAN and GNSS to predict a potential crash and alert the driver. V2V messages could also potentially be fused with on-board sensors like Radar, Lidar, and Camera to improve the confidence level of vehicle detection for safety applications or even for autonomous driving to some extent.

It is important to understand that V2V can avoid only the crashes involving more than one vehicle. The primary motivation behind mandating the use of V2V based technology is the number of crashes estimated by NHTSA that can be avoided by this technology. 62% of all the crashes (approximately 3.4 million) are light-vehicle to light-vehicle crashes. The economic and comprehensive costs for these crashes amount to approximately \$109 billion and \$319 billion, respectively [2]. NHTSA has performed analysis using data from 2010 through 2013 and came up with top 10 pre-crash scenarios (listed in Table 1) that can be addressed by V2V. It was then determined that these 10 scenarios could be addressed by the following six safety applications: (1) Forward Collision Warning (FCW), (2) Electronic Emergency Brake Light (EEBL), (3) Intersection Move Assist (IMA), (4) Do Not Pass Warning (DNPW), (5) Blind Spot Warning/Lane Change Warning (BSW/LCW), and (6) Left Turn Assist (LTA). These applications proved to mitigate and prevent potential crashes in the Connected Vehicle Safety Pilot Deployment Program conducted by University of Michigan Transportation Research Institute (UMTRI).

Table 1: Safety Applications associated with Pre-Crash Scenarios

| Pre-Crash Scenarios                          | Pre-Crash Group       | Associated Safety Application                              |
|----------------------------------------------|-----------------------|------------------------------------------------------------|
| Lead Vehicle Stopped                         | Rear-end              | Forward Collision Warning                                  |
| Lead Vehicle Moving                          | Rear-end              | Forward Collision Warning                                  |
| Lead Vehicle Decelerating                    | Rear-end              | Forward Collision Warning/Emergency Electronic Brake Light |
| Straight Crossing Path without traffic light | Junction Crossing     | Intersection Movement Assist                               |
| Left-Turn Across Path/Opposite Direction     | Left Turn at Crossing | Left Turn Assist                                           |
| Opposite Direction/No Maneuver               | Opposite Direction    | Do Not Pass Warning                                        |
| Opposite Direction/Maneuver                  | Opposite Direction    | Do Not Pass Warning                                        |
| Change Lane/Same Direction                   | Lane Change           | Blind Spot Warning/Lane Change Warning                     |
| Turning/Same Direction                       | Lane Change           | Blind Spot Warning/Lane Change Warning                     |
| Drifting/Same Direction                      | Lane Change           | Blind Spot Warning/Lane Change Warning                     |

## Contents

1. [DSRC Protocol Stack and Underlying Standards](#dsrc-protocol-stack-and-underlying-standards)
1. [System Architecture](#system-architecture)
1. [Program Flow and Required Components for V2V Safety Applications](#program-flow-and-required-components-for-v2v-safety-applications)
    - [Path History](#path-history)
    - [Host Vehicle Path Prediction (HVPP)](#host-vehicle-path-prediction-hvpp)
    - [Target Classification (TC)](#target-classification-tc)
1. [V2V Safety Applications](##target-classification-tc)
    - [Forward Collision Warning (FCW)](#forward-collision-warning-fcw)
    - [Electronic Emergency Brake Light (EEBL)](#electronic-emergency-brake-light-eebl)
    - [Intersection Movement Assist (IMA)](#intersection-movement-assist-ima)
    - [Do Not Pass Warning (DNPW)](#do-not-pass-warning-dnpw)
    - [Blind-Spot Warning (BSW)/Lane-Change Warning (LCW)](#blind-spot-warning-bswlane-change-warning-lcw)
    - [Left Turn Assist (LTA)](#left-turn-assist-lta)
    - [Control Loss Warning (CLW)](#control-loss-warning-clw)

## DSRC Protocol Stack and Underlying Standards
Figure 1 illustrates the DSRC protocol stack and related standards. The standards define how data is communicated and interpreted from one V2V device to another device.

<p align="center">
  <img src="/docs/v2x/img/protocol-stack.png">
</p>
<center>Figure 1: DSRC Protocol Stack</center>  

The bottom layer in the stack starts at the DSRC Radio level and indicates how the raw data is transmitted over the V2V wireless network. This layer also provides the received raw information to the next layer, which then organizes the data packets into network frames. These two layers are covered by IEEE 802.11p [10]. IEEE 1609 [2, 6-9] WAVE stack builds on IEEE 802.11p and defines the higher layer standards. SAE J2735 Standard specifies message sets, data frames, and data elements, specifically for use by applications intended to utilize the 5.9 GHz DSRC for WAVE communications systems [1]. And, SAE J2945/1 specifies the system requirements for an on-board vehicle-to-vehicle (V2V) safety communications system for light vehicles, including standards profiles, functional requirements, and performance requirements [11].

## System Architecture

Figure 2 shows the system architecture of a typical DSRC implementation based V2V On-Board System. At minimum, the hardware includes a processor, a DSRC radio for the transmission and reception of V2V messages, a GPS module for the reception of location information, and few peripherals and interfaces for supporting the entire system. On-Board Unit uses CAN modules to bring in vehicle data such as speed, yaw rate, and steering-wheel angle. Hardware Security Module (HSM) is another important module OBU needs to have for managing a vehicle’s security certificates and guarding against equipment tampering and bus probing. Some of the GPS units have an Inertial Management Unit (IMU) integrated or an interface to connect an IMU and can provide information like velocity, roll, pitch, and heading.

The Board Support Package (BSP) is the software responsible for hardware specific operations required to get the OS up and running. The BSP and drivers provide support for the hardware components and a way for the Middleware Stack to access them. Middleware Stack takes care of the upper layer standards to provide message encoding and decoding, signing and verifying BSMs, network and transport services and so on. It provides the data from received V2V message to the application layer in a way that can be understood by an application developer. It also provides Application Program Interfaces (API) to the application layer for using the lower layer services and makes it easier and straightforward for an application engineer to do the development.

<p align="center">
  <img src="/docs/v2x/img/architecture.png">
</p>
<center>Figure 2: DSRC based V2V On-Board System Architecture</center>  

## Program Flow and Required Components for V2V Safety Applications

Figure 3 shows a typical program flow at the application layer for the implementation of V2V Safety applications. It shows two major processes in a multithreaded environment. For broadcasting a BSM, DSRC stacks might either provide a handle to a callback function every 100ms (or the time duration specified between two BSMs) or a timer can be setup that expires every 100ms and calls a function. The Host Vehicle data can then be retrieved from interfaces like GPS and CAN (The values of some data elements like Path History (PH) need to be calculated). Once all the data is available, the Basic Safety Message is constructed in accordance to [1]. The message is then encoded at each layer before being transmitted over DSRC by the stack based on the ASN.1 packet encoding rules [12] specified by the standards (For example - SAE J2735 specifies UPER ASN encoding of the payload).

<p align="center">
  <img src="/docs/v2x/img/program-flow.png">
</p>
<center>Figure 3: V2V Safety Applications Program Flow</center>  

The other part of the program flow shown above takes care of the received BSM and determination of collision possibility. The DSRC software stack provides a callback function that is executed every time a BSM is received. The stack also makes a structure containing all the BSM data elements corresponding to the Remote Vehicle (RV) available for use by the application developer. Host vehicle (HV) data can be obtained from interfaces on-board like GPS and CAN. The algorithms to detect a possibility of collision can be executed once the Host Vehicle and Remote Vehicle data is available. One of the major required components for any safety application is Target Classification which basically classifies the position and orientation of Remote Vehicle relative to the Host Vehicle. Target Classification would further need to perform some operations like Path Prediction Radius Calculation and Path Prediction Confidence Calculations. Path History can also be used to improve the Target Classification since an extrapolation of the past maneuver can provide prediction for future maneuvers with some confidence.

Based on the Remote Vehicle relative zone and direction of travel, the relevant safety applications may be executed (e.g. If the Remote Vehicle relative zone is “Ahead Left” and relative direction is “Reverse”, then the only safety applications that would be relevant are Do Not Pass Warning (DNPW) and Left Turn Assist(LTA)). Once we have result from all the safety applications, the highest priority warning can be provided to the driver on User Interface (e.g. Forward Collision Warning (FCW)will have a higher priority than Emergency Electronic Brake Light (EEBL) because FCW has a lower time-to-collision).

#### Path History

Path History (PH) represents the HV actual path with a set of concise data elements. The concise data elements are a sampled subset of the actual elements which are selected such that the perpendicular distance between any point on the actual vehicle path and the chord connecting two concise points is less than a calibration parameter (1 meter). The size of the buffer containing the concise data elements is such that the represented PH distance computed using the elements of the buffer is at least a certain minimum length defined by the calibration parameter (300 meters) [2]. The standard SAE J2945/1 specifies three methods for computing the concise path history points.

Figure 4 shows a plot of actual vehicle path and the concise path history data elements taking up to 300 actual vehicle data elements in the buffer for computation.

<p align="center">
  <img src="/docs/v2x/img/path-history.jpg">
</p>
<center>Figure 4: Vehicle Actual Path and Concise Path History Data Elements</center>   

[![PH Demo](/docs/v2x/img/ph-demo.png)](https://youtu.be/YnJhjDUXc8A "PH Demo")  

#### Host Vehicle Path Prediction (HVPP)  

Path Prediction is a way to determine where the vehicle is headed. Predicting the future trajectories of Host Vehicle can be very useful for classifying the relative position of Remote Vehicles. The trajectory can be represented as a circle of radius, R, and an origin located at (0, R). The radius, R is positive for curvatures to the right from the transmitting vehicle’s perspective. A radius value of 32,767 is interpreted as a straight path.

It is possible to estimate the future trajectories from vehicle dynamics. The radius of curvature can be computed based on the HV speed and the rate of change of heading (yaw rate). This curvature can then be extrapolated forward to provide an estimate of the likely future path of the vehicle. Sign of the radius is positive if vehicle is moving in clockwise direction and negative if vehicle is moving in anti-clockwise direction.

#### Target Classification (TC)

Target Classification provides a 360 degrees, relative lane-level classification of the locations and direction of travel of communicating Remote Vehicle’s relative to Host Vehicle. Output of the TC module include relative position, relative direction of travel, lateral offset, longitude offset, and delta heading. It uses vehicle information like latitude, longitude, elevation, speed, heading, yaw rate, path history, and path prediction objects from the Basic Safety Message (BSM).

Figure 5 shows the RV Zones relative to the HV. The TC module can classify the RV position as one of the 14 zones shown in figure 5. The module also provides relative RV direction of travel as one of the four possible directions shown in figure 6.

<p align="center">
  <img src="/docs/v2x/img/tc-zones.png">
</p>
<center>Figure 5: RV position (Zone) relative to the HV</center>

<p align="center">
  <img src="/docs/v2x/img/tc-dir.png">
</p>
<center>Figure 6: RV direction of travel relative to the HV</center>

Based on the relative zones and direction obtained from the TC module, relevant safety applications are executed and their outputs are monitored. The warning from highest priority safety application is then presented to the driver over an HMI. Figure 7 shows the relevant Safety Application for various relative zone and direction obtained from the TC module.

<p align="center">
  <img src="/docs/v2x/img/apps-mapping.png">
</p>
<center>Figure 7: Mapping of Safety Applications to the output of TC module</center>

## V2V Safety Applications

Vehicle position and dynamics information like latitude, longitude, elevation, heading, speed, yaw rate etc. can be used to predict the future path of vehicles and estimate the possibility of collision in near future. The safety applications considered in this section address rear-end, opposite direction, junction crossing, and lane change crash scenarios. These scenarios can be covered by the safety applications listed in Table 1 and are described below.

#### Forward Collision Warning (FCW)

FCW warns the driver of an impending collision between the HV front-end and a RV rear-end. This collision is possible only if both the HV and RV are driving in the same direction and are on the same lane (RV could be stopped, decelerating, or moving at a speed slower than the HV).

The output of Target Classification (TC) Module is used to determine if RV is in the same lane as the HV. TC also provides the longitudinal offset which is used along with vehicle dynamics information to determine if there is a possibility of forward collision. The simplest way to predict forward-collision is to compute the time-to-collision (TTC) and compare that with a calibrated threshold value.

<p align="center">
  <img src="/docs/v2x/img/fcw.png">
</p>
<center>Figure 8: (a) FCW Target Classification Zones &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; (b) An example scenario for FCW</center>

#### Electronic Emergency Brake Light (EEBL)

EEBL addresses the scenario where a Remote Vehicle is ahead and in the same lane as Host Vehicle. If there are other vehicles between HV and RV considered, and the RV brakes hard, a potential crash is imminent if no brake light is visible on the vehicle in front of HV. EEBL can issue a warning to the driver of HV in such scenarios where the RV is not directly visible to the HV and thus can help avoid the crash.

<p align="center">
  <img src="/docs/v2x/img/eebl.png">
</p>
<center>Figure 9: (a) EEBL Target Classification Zones &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; (b) An example scenario for EEBL</center>

#### Intersection Movement Assist (IMA)

IMA safety application is intended to warn the driver of HV if it is not safe for the HV to enter an intersection due to high possibility of collision with other RVs. This application estimates the time taken by the HV and RV to arrive at the intersection point and issues a warning if both vehicles are predicted to arrive at approximately the same time.

This module can issue either “IMA Right” or “IMA Left” warning based vehicle position, speed, heading information. Target Classification Zone and Direction outputs are used to determine if a Remote Vehicle is in either IMA Right or IMA Left Zone.

<p align="center">
  <img src="/docs/v2x/img/ima.png">
</p>
<center>Figure 10: (a) IMA Target Classification Zones &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; (b) An example scenario for IMA</center>

#### Do Not Pass Warning (DNPW)

DNPW is intended to warn the driver of HV during a passing maneuver attempt when a slower-moving vehicle ahead cannot be passed safely using a passing zone, because the passing zone is occupied by vehicles moving in the opposite direction. This module can either provide advisory information or a warning based on the driver’s intent to overtake. The arbitration can be done by observing the left-turn activation signal over vehicle CAN.

<p align="center">
  <img src="/docs/v2x/img/dnpw.png">
</p>
<center>Figure 11: (a) DNPW Target Classification Zones &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; (b) An example scenario for DNPW</center>

#### Blind-Spot Warning (BSW)/Lane-Change Warning (LCW)

BSW safety application either provides an advisory alert or warns the driver of HV if another vehicle occupies the adjacent lane in Host Vehicle’s Blind-Spot. The application arbitrates one versus the other depending on driver’s intent to change the lane to the one occupied by another vehicle. A driver shows the intention to change the lane by activating a turn signal which information can be pulled into the system from the Vehicle CAN Bus.

An extension of the Blind-Spot Warning is the Lane-Change Warning which generates a warning if the HV’s blind-spot is not currently occupied by a Remote Vehicle but will soon be occupied by a fast approaching vehicle and a collision is inevitable if the HV changes its lane. Clearly, this application module is relevant only if the RV is travelling in the same direction on an adjacent lane and the driver of HV intends to change the lane.

<p align="center">
  <img src="/docs/v2x/img/bsw.png">
</p>
<center>Figure 12: (a) BSW/LCW Target Classification Zones &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; (b) An example scenario for BSW/LCW</center>

#### Left Turn Assist (LTA)

The LTA module assists the driver of HV in making a left turning maneuver at an intersection. It can use both V2V and V2I information to decide if it is safe to turn left at an intersection. Two types of messages, Map Data (MAP) & Signal Phase and Time (SPaT) are broadcasted from the Roadside Unit (RSU). Typically, MAP is sent every second and SPaT every 100 ms. MAP describes the physical geometry of one or more intersections. The SPaT message informs drivers about the current status and change of the traffic signal ahead as well as when the next signal stage changes [14]. It also provides information about approaching traffic to optimize the signal system. The IntersectionGeometryData Frame in the MAP message combined with the vehicle position can be used to determine if HV is in a lane for which left maneuver is allowed. The SPaT message provides the traffic light status (in MovementPhaseStateData Element of the SPaT message) for the lane that the HV is currently in.

This module generates a warning if a RV is approaching fast towards the HV from ahead while HV is in a lane that has left maneuvers allowed and the Traffic Light State is one of the following: “permissive-green”, “permissive-yellow”, “protected-yellow”, or “flashing-yellow” [1].The distance needed to be travelled by the HV to reach the intersection can be easily computed using the intersection geometry information in the MAP message and HV current position, which then further combined with a calibrated distance required to make a turn yields the total distance from HV to the road it needs to go on after making a turn.

<p align="center">
  <img src="/docs/v2x/img/lta.png">
</p>
<center>Figure 13: (a) LTA Target Classification Zones &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; (b) An example scenario for LTA</center>

#### Control Loss Warning (CLW)

The CLW module warns the driver of the HV if any RV ahead and in the same or adjacent lane as HV loses its control. A control loss event is defined as the activation of one of the following: Antilock Brake System (eventABSactivated), Traction Control Loss (eventTractionControlLoss), or Stability Control Loss (eventStabilityControlactivated). If RV detects a control loss event, it sets the corresponding flag in the Data Element DE_VehicleEventFlags [1] and broadcasts this information within the BSM.

A RV is relevant for the CLW application if it is ahead of the HV in either the same or adjacent lane as the HV, and its relative direction is either Equidirectional or Reverse. Upon receiving a BSM from the RV, if HV determines that the vehicle is in CLW Zone and one of the control loss event flag is set, then a warning is issued if calculated TTC is lower than a calibrated threshold.

<p align="center">
  <img src="/docs/v2x/img/clw.png">
</p>
<center>Figure 14: (a) CLW Target Classification Zones &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; (b) An example scenario for CLW</center>

### References

[1] SAE International, “Dedicated Short Range Communications (DSRC) Message Set Dictionary” SAE J2735 Standard, March 2016.  
[2] National Highway Traffic Safety Administration (NHTSA), Department of Transportation (DOT), "Notice of proposed rulemaking (NPRM)," Federal Motor Vehicle Safety Standards; V2V Communications, Tech. Rep. 49 CFR Part 571, 2016.  
[3] IEEE standard for wireless access in vehicular environments--security services for applications and management messages. IEEE Std 1609. 2-2016 (Revision of IEEE Std 1609. 2-2013) pp. 1-240. 2016. DOI: 10.1109/IEEESTD.2016.7426684.  
[4] M. Raya and J. Hubaux, “The Security of Vehicular Networks,” EPFL Technical Report IC/2005/009, March 2005.   
[5] CAMP LLC - Vehicle Safety Communication 5 (VSC5), "Security Credential Management System Proof–of–Concept Implementation - "EE Requirements and Specifications Supporting SCMS Software Release 1.0"," January 11, 2016.  
[6] IEEE guide for wireless access in vehicular environments (WAVE) - architecture. IEEE Std 1609. 0-2013 pp. 1-78. 2014. . DOI: 10.1109/IEEESTD.2014.6755433.  
[7] IEEE standard for wireless access in vehicular environments (WAVE) -- networking services. IEEE Std 1609. 3-2016 (Revision of IEEE Std 1609. 3-2010) pp. 1-160. 2016. . DOI: 10.1109/IEEESTD.2016.7458115.  
[8] IEEE standard for wireless access in vehicular environments (WAVE) -- multi-channel operation. IEEE Std 1609. 4-2016 (Revision of IEEE Std 1609. 4-2010) pp. 1-94. 2016. . DOI: 10.1109/IEEESTD.2016.7435228.  
[9] IEEE standard for wireless access in vehicular environments (WAVE) - identifier allocations. IEEE Std 1609. 12-2016 (Revision of IEEE Std 1609. 12-2012) pp. 1-21. 2016. . DOI: 10.1109/IEEESTD.2016.7428792.  
[10] IEEE standard for information technology-- local and metropolitan area networks-- specific requirements-- part 11: Wireless LAN medium access control (MAC) and physical layer (PHY) specifications amendment 6: Wireless access in vehicular environments. IEEE Std 802. 11p-2010 (Amendment to IEEE Std 802. 11-2007 as Amended by IEEE Std 802. 11k-2008, IEEE Std 802. 11r-2008, IEEE Std 802. 11y-2008, IEEE Std 802. 11n-2009, and IEEE Std 802. 11w-2009) pp. 1-51. 2010. . DOI: 10.1109/IEEESTD.2010.5514475.  
[11] SAE International, “On-Board System Requirements for V2V Safety Communications” SAE J2945/1 Standard, March 2016.  
[12] ITU-T: OSI networking and system aspects – Abstract Syntax Notation One (ASN.1), "Information technology – ASN.1 encoding rules: Specification of Packed Encoding Rules (PER)".  
[13] Harding, J., Powell, G., R., Yoon, R., Fikentscher, J., Doyle, C., Sade, D., Lukuc, M., Simons, J., & Wang, J. (2014, August). Vehicle-to-vehicle communications: Readiness of V2V technology for application. (Report No. DOT HS 812 014). Washington, DC: National Highway Traf c Safety Administration.   
[14] ISO/TC 204/SC/WG 18, Intelligent transport systems — Cooperative Systems — SPAT (Signal Phase and Timing) message, MAP (Intersection topology) message, Draft versions 2013.  
