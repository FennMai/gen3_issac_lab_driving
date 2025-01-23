# Gen3 Isaac Lab Driving :robot:

[![License: BSD-3-Clause](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![Isaac Lab Version](https://img.shields.io/badge/Isaac_Lab-2024.1-blue.svg)](https://github.com/isaac-sim/IsaacLab)
[![Kinova Gen3](https://img.shields.io/badge/Kinova-Gen3_7DoF-red.svg)](https://www.kinovarobotics.com/product/gen3-robots)

<div align="center">
[English](./README.md) | [中文](./README_Ch.md)
</div>

An open-source research for Kinova Gen3 manipulator control in Isaac Lab environment.



## :warning: Important Disclaimer

**This project is not affiliated with or endorsed by NVIDIA Corporation, Kinova Inc., or any of their subsidiaries. All referenced trademarks and product names remain the property of their respective owners.**

**Educational Use Only**  
This repository contains material intended for educational and research purposes only. Users assume full responsibility for:
1. Compliance with Kinova Gen3 hardware operational guidelines
2. Adherence to Isaac Lab's End User License Agreement (EULA)
3. Any modifications to physical hardware systems

**No Warranty**  
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND. THE AUTHOR SHALL NOT BE HELD LIABLE FOR ANY DAMAGES ARISING FROM THE USE OF THIS REPOSITORY.

**Hardware Safety Notice**  
Always maintain emergency stop access when operating physical robotic systems. Verify all control commands in simulation before hardware deployment.

---

## :rocket: Features

- **Gen3 Differential Inverse Kinematics Control**  
  `basic_control/BasicScene_Gen3_Control.py`  
  - Cartesian space end-effector control
  - Damped Least Squares (DLS) solver
  - Multi-environment simulation support
  
![alt text](notes/gen3_control_scene.gif)

## :chart_with_upwards_trend: Roadmap

### Short-Term Goals
- [ ] Robotiq 2F-85 Gripper Integration
- [ ] Contact-rich manipulation scenarios
- [ ] Tactile data collection framework

### Long-Term Vision
- [ ] Reinforcement Learning pipeline
- [ ] Sim2Real transfer modules
- [ ] Multi-modal perception system

---

## :books: Learning Resources

### Core Technologies
| Resource | Description |
|----------|-------------|
| [Isaac Lab Docs](https://docs.isaac.lab) | Official simulation platform documentation |
| [Kortex API](https://github.com/Kinovarobotics/ros_kortex) | Kinova official ROS interface |
| [Compliant Control](https://github.com/empriselab/gen3_compliant_controllers) | Force-sensitive manipulation implementations |
|[Manipulation Experiments](https://github.com/kyassini/manipulation_experiments) |Advanced control strategies

