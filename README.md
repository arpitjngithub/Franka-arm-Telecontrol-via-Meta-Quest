# Franka-arm-Telecontrol-via-Meta-Quest
 
## Real-Time VR Teleoperation of a Franka Arm for Construction Automation and Human-Robot Interaction

This repository contains the full codebase, setup instructions, and supporting files for a real-time, Linux-based VR teleoperation system. Built during a research internship at the University of Alberta, the system enables precise and intuitive control of a Franka Emika Research 3 robot arm using a Meta Quest 2 headset.

- **Internship**: Civil and Environmental Engineering, University of Alberta, Canada  
- **Intern**: Arpit Jain (IIT Bombay)  
- **Supervisors**: Yuezhen Gao, Prof. Dr. Qipei Mei  

---

##  System Overview

The project demonstrates an open-source, low-latency telecontrol pipeline with:
- Unity-based Meta Quest 2 pose streaming (6DoF) via ALVR + SteamVR
- ROS 2 Humble running on a Jetson Nano (real-time kernel)
- Real-time control of the Franka Emika arm using `franka_ros2` and custom ROS 2 nodes

---

##  Key Technologies

- **Hardware**: Meta Quest 2, Franka Emika FR3, Jetson Orin Nano
- **Software**: ROS 2 Humble, Unity 2022, ALVR, SteamVR, libfranka
- **Networking**: USB-C for headset tracking, Ethernet between Jetson and Franka

---

##  What This System Solves

-  Eliminates reliance on expensive, Windows-only VR toolchains  
-  Fully Linux-based, low-latency XR control (< 20 ms)
-  Precise pose tracking for millimeter-level brick and sensor placement
-  Easily reproducible by other labs for training or site automation

---

##  Quick Demo & Poster

-  [Final Demo Video](https://drive.google.com/file/d/1djWV3_R8KXjHr3tL3xHfMX5mXeVSdfsM/view?usp=sharing)  
-  [Technical Poster (PDF)](https://drive.google.com/file/d/1nZMoYLV-StBFFC5Pdl3KX6oBUIysdNmN/view?usp=sharing)  
-  [Complete Setup Video](https://drive.google.com/file/d/1e0ziVf_1DyKf7BcEupy_euwdH6GXxAyO/view)

---

##  Full Documentation

See [`Internship_Documentation.pdf`](Internship_Documentation.pdf) for:

- System architecture & networking
- Meta Quest 2 + ALVR + SteamVR setup (Linux)
- ROS 2 & libfranka package structure
- Real-time kernel tuning
- Cartesian pose controllers
- Custom C++ motion tests
- Troubleshooting & recovery steps

---

##  Future Work

- Add eye-in-hand camera & vision-language model for commands like “stack this” or “hand me the wrench”
- Develop semi-autonomous macros for precise tool delivery or placement
- Integrate on-site progress logging for BIM-based construction monitoring

---

##  Acknowledgements

This project was conducted under the guidance of **Yuezhen Gao** and **Prof. Dr. Qipei Mei** in the Civil & Environmental Engineering Department, University of Alberta, as part of the 2025 UARE research internship program.

For inquiries: [arpit.jain1@iitb.ac.in](mailto:arpit.jain1@iitb.ac.in)
