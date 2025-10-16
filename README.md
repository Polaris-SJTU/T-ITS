# Leader-Following Formation Control for Cooperative Transportation of Multiple Mecanum-Wheeled Mobile Robots

This repository contains the code and experimental data for the cooperative transportation system of **Mecanum-wheeled robots** based on the **leader-follower formation control** strategy. The work demonstrates the application of a decentralized, communication-free control framework for multi-robot cooperation in complex environments. The experimental validation of the proposed method is shown through a series of controlled tests involving multiple robots performing formation tracking and cooperative transport tasks.

## Overview

The proposed control system allows multiple Mecanum-wheeled robots to collaborate in a leader-follower formation without relying on global positioning systems (GPS) or inter-robot communication. The robots rely on **local relative measurements** from onboard sensors (e.g., cameras, LiDAR) to maintain formation and transport tasks. This method is particularly suited for applications in **constrained environments** where traditional GPS-based systems may fail or where communication is limited.

## Experimental Setup

The experiments were conducted using **three robots** in a controlled laboratory environment. The validation relies exclusively on onboard sensors to prove the system's practical feasibility.

| ![Experimental Setup](./data/payload_setup.png) | ![Experimental Setup](./data/onboard_sensor_setup.png) |
|:-----------------------------------------------:|:------------------------------------------------------:|
|               (a) Payload setup.                |               (b) Onboard sensor setup.                |

*Figure 1: Experimental setup and environment.*

The setup consists of:

  * **Onboard Sensors**: Each robot is equipped with a **1080P fisheye camera** and a **SLAMTEC A1M8 LiDAR**.
  * **Visual Tracking**: The leader robot is fitted with an **ArUco tag**, allowing for robust visual detection and pose estimation by the followers' cameras.
  * **Sensor Fusion**: A **Kalman filter** is implemented on each follower to fuse the data from the camera (providing accurate orientation and lateral position) and the LiDAR (providing accurate distance), generating a stable and high-fidelity estimate of the leader's relative pose.
  * **Payload**: A transparent rectangular frame is used as the payload to allow for a clear, unobstructed view of the robots' formation during transport maneuvers.

### Experimental Video

You can view the full experimental video demonstrating the robots’ operation and coordination during the cooperative transport task:

https://github.com/user-attachments/assets/448016f7-7e90-4010-9c91-1b2565127c7e

*Video 1: Experimental video 1.*

https://github.com/user-attachments/assets/5c5b619d-2105-4a31-b25c-afc4c052fd0d

*Video 2: Experimental video 2.*

## Features
* **Rhombic-Dodecahedral Velocity Constraints Modeling**: First geometric characterization of Mecanum-wheeled robots' velocity limits as a **rhombic dodecahedron**, enabling safe and precise motion planning in confined spaces.  

* **Communication-Free Decentralized Control**: Leader-follower formation control **without inter-robot comms or global positioning** – each robot acts solely on local sensor data (e.g., LiDAR/camera).  

* **Disturbance-Robust Tracking via Velocity Observer**: Real-time leader velocity estimation compensates slippage/heading deviations, guaranteeing **bounded formation errors** under payload disturbances.  

## Code

The system is implemented through a multi-layer architecture:
- **Upper Layer (ROS-based Perception, Decision & Control)**  
  Implemented in **Python and C++** using ROS (Robot Operating System) for:
  - Real-time perception processing
  - Formation decision-making
  - Leader-follower control logic
- **Lower Layer (Robot Drive Control)**  
  Implemented in **Arduino C++** for:
  - Low-level Mecanum wheel actuation
  - Motor control and odometry processing

The repository includes:
* **Controller Implementation**: Code for the decentralized leader-follower control strategy.
* **Observer Design**: Implementation of the velocity observer to estimate the leader’s velocity.
* **Experimental Scripts**: Scripts to validate the system’s performance in controlled experiments.

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](./LICENSE) file for details.