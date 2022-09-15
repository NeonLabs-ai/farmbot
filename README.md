## Modular autonomous mobile robot for precision spray of Pesticides

![GitHub Repo stars](https://img.shields.io/github/stars/Neonlabs-ai/farmbot?color=FFE333)
![GitHub Repo forks](https://img.shields.io/github/forks/Neonlabs-ai/farmbot?color=FFE333)

![Ubuntu](https://img.shields.io/badge/OS-Ubuntu-informational?style=flat&logo=ubuntu&logoColor=white&color=2bbc8a)
![ROS](https://img.shields.io/badge/Tools-ROS-informational?style=flat&logo=ROS&logoColor=white&color=2bbc8a)
![C++](https://img.shields.io/badge/Code-C++-informational?style=flat&logo=c%2B%2B&logoColor=white&color=2bbc8a)

## TODO

- [ ] CAD Design of robot (Must be optimized for manufacturing)
- [x] Program localisation method using GPS, IMU (and Vision if required)
- [x] Research about architecture of CNN to be used..
- [x] Train for weed detection purpose
- [ ] define and solve lane tracking problem as input image and output Lane msg (to be modified)
- [ ] deploy CNN in Jetson / Pi for weed detection purpose using TensorRT
- [ ] Nozzle position planning algorithm

### Description

A farming robot with automatic navigation, weed detection system, designed for
navigating between crops, across farms and providing aid in multiple operations
required by farmers, specific in automatic weed spraying system.
Automatic weed detection and localization is achieved with deep neural network
running on NVIDIA jetson nano, then planning sprayer path and spraying precisely
on weeds. It is developed using ROS as its core, leveraging the open-source community to each components which makes system
modular, efficient and scalable. 
GNSS and RTK based approach is used for localization, and for increasing
accuracy, sensor fusion method is used for more accurate state estimation.
Localization is then used with vision system to guide through crop lanes and then
deep neural network running parallel to lane tracker program detects weeds with
locations and plans trajectory of nozzle and spraying time stamp and controller will
execute planned trajectory.

### Objectives
1. Reduce amount of chemical (weedicide) usage in crops using automatic high
precision weed detection (using CNN) and low-cost spraying system.
2. Develop RTK based automatic navigation system for accurate positioning and
motion across different lanes of crop field.
3. Develop base platform with modular system for further improvisation in multi -
functionality of robot (future work).

### Major Scientific fields of Interest

Such autonomous precise weedicide spraying robots are available in market but are
too expensive because they run on heavy Deep Neural Networks, which are GPU-
intensive and it’s not really needed because the risk factor is less so the trade-off of
accuracy and speed can be favored more towards speed and therefore we would
be using a convolutional network with lesser number of layers. Since the number of
classes here are less (say, crops, weeds and lane), A narrow neural network would
still work, which would even work on a common small-scale robotic system viz,
Raspberry Pi and NVIDIA Jetson Nano.

### Approach

* Vision and GNNS + RTK based autonomous navigation of vehicle.
* Development of ecosystem of modular systems using robot operating system
(ROS) for farming application.
* Design of vehicle with certain payload capacity and mobility.
Vision based weed and crop detection, classification, localization and precisely spraying over weeds.
* Robot operating system will interface with micro-controllers to control vehicle trajectory, sprayer nozzle position and get sensor data.
* Localization of vehicle using GNSS + RTK based system and implementing sensor fusion method to improve state estimation of vehicle.

### FLOWCHART :

![](https://github.com/bhavikmk/farmbot/blob/main/assets/flowchart.png)

### 3D Model of the Farmbot

![](https://github.com/akgcode/farmbot/blob/main/assets/rendered_farmbot_1.JPG)
![](https://github.com/akgcode/farmbot/blob/main/assets/snap_farmbot_1.jpg)

<!-- ## Name of equipment and accessories required for R&D

1. MXG Vector signal generator 100kHz-3GHz : N5182A
2. Cognitive Wireless communication SDR Lab : software
3. RF signal generator 9KHz to 3.0 GHz : N9310A
4. Frequency range 9 KHz to 3.0GHz CXA signal : N9000A
analyzer
5. Programmable Function Generator 25 MHz
Single Channel : Keysight-3352B
6. Digital storage oscilloscope : DSOX2022A
7. 100 MHz Digital storage oscilloscope : TBS1102B-EDU
8. 150/200 MHz Digital storage oscilloscope : TBS1152B-EDU
9. Wireless Measurement studio (Field fox VNA) 
10. N5172B EXG X-Series RF vector signal
generator: Keysight
11. ARAMIS GPS L1, IRNSS L5, SDR Receiver : IP Solutions
12. Accord NavIC Receiver : Accord -->


<!-- ## Fund Requirement
Detailed break-up for the Project budget should be given as follows:
Total
Consumables
& Supplies
Hardware Price

GNSS and RTK based motion
> GNSS Receiver Module (x2) : 12,000

Weed Detection System

> Jetson Nano 4GB RAM : 25,000
> Image Sensor (Pixy 2.1 camera) : 8,000


Spraying Mechanism

> GT2 Timing Belt and Pulley(x2) :500*2 = 1000
> Hollow-cone Nozzle (60 deg & 90 deg) : 500*2 = 1000
> Garden Sprayer with Tank (16L) : 4000
> NEMA17 1.6 kg-cm Stepper Motor :1,000 
Total : 7,000

Vehicle Motion

> High power High torque 24V brushed 
> planetary gear motor. Model 60PG60S : 20,000*2=40 
> Photoelectric Speed Sensor Encoder (2) : 2000
> DC Motor controller 250W : 3000
Total : 45,000

**Overall total : 117,000** -->

### References
- Meshram, A. T., Vanalkar, A. V., Kalambe, K. B., & Badar, A. M. (2022). Pesticide Spraying Robot for Precision Agriculture: A Categorical Literature Review and Future Trends. Journal of Field Robotics, 39, 153–171. [LINK](https://doi.org/10.1002/rob.22043) 
