# Gen3 Isaac Lab 驱动控制 :robot:

[![许可证: BSD-3-Clause](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![Isaac Lab 版本](https://img.shields.io/badge/Isaac_Lab-2024.1-blue.svg)](https://github.com/isaac-sim/IsaacLab)
[![Kinova Gen3](https://img.shields.io/badge/Kinova-Gen3_7DoF-red.svg)](https://www.kinovarobotics.com/product/gen3-robots)

<center>

[English](./README.md) | [中文](./README_Ch.md)

</center>


基于 Isaac Lab 环境的 Kinova Gen3 机械臂控制开源研究平台

## :warning: 重要免责声明

**本项目与 NVIDIA Corporation、Kinova Inc. 及其子公司无任何关联或授权。所有提及的商标和产品名称均为其各自所有者的财产。**

**仅供教育使用**  
本仓库材料仅用于教育和研究目的。使用者需自行承担：
1. 遵守 Kinova Gen3 硬件操作指南
2. 遵守 Isaac Lab 最终用户许可协议 (EULA)
3. 对物理硬件系统的任何修改

**无担保声明**  
本软件按"原样"提供，无任何形式的担保。作者不对因使用本仓库导致的任何损害承担责任。

**硬件安全声明**  
操作实体机器人系统时，请始终确保紧急停止功能的可用性。硬件部署前请先在仿真环境中验证所有控制指令。

---

## :rocket: 已实现功能

- **Gen3 微分逆运动学控制**  
  `basic_control/BasicScene_Gen3_Control.py`  
  - 笛卡尔空间末端执行器控制
  - 阻尼最小二乘法 (DLS) 求解器
  - 多环境仿真支持

![alt text](notes/gen3_control_scene.gif)

---

## :chart_with_upwards_trend: 开发路线

### 短期目标
- [ ] 集成 Robotiq 2F-85 夹爪
- [ ] 配置接触式操作场景
- [ ] 搭建触觉数据采集框架

### 长期愿景
- [ ] 强化学习全流程实现
- [ ] 仿真到实物迁移模块
- [ ] 多模态感知系统

---

## :books: 学习资源

### 核心技术
| 资源 | 描述 |
|------|------|
| [Isaac Lab 文档](https://docs.isaac.lab) | 官方仿真平台文档 |
| [Kortex API](https://github.com/Kinovarobotics/ros_kortex) | Kinova 官方 ROS 接口 |
| [柔顺控制](https://github.com/empriselab/gen3_compliant_controllers) | 力敏感操作实现 |
| [操作实验](https://github.com/kyassini/manipulation_experiments) | 先进控制策略 |


---

## :balance_scale: 许可协议
BSD 3-Clause License - 完整文本请查看 [LICENSE](LICENSE)