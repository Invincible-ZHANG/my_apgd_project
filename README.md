# my_apgd_project

Implementation of an Accelerated Projected Gradient Descent (APGD) Solver for Multibody Dynamics Simulation  
多体动力学仿真的加速投影梯度下降（APGD）求解器实现

Author
Zijian Zhang / 张子健

Email: zhangzijiandavid@outlook.com


---

## Overview

This project implements an **Accelerated Projected Gradient Descent (APGD)** solver, specifically designed for **multibody dynamics (MBD) simulation** with complementarity constraints (CCP).  
It is intended as a reference and research implementation for academic and engineering purposes.

本项目实现了适用于多体动力学补偿约束(CCP)问题的**加速投影梯度下降（APGD）数值求解器**，可作为算法学习、工程集成或学术研究的基础代码。


---
## 文件结构

my_apgd_project/
├── .vs/ # Visual Studio 工程文件夹
├── solver/ # 源码主目录
│ ├── include/ # 头文件（接口、声明等）
│ ├── src/ # 源文件（主要实现）
│ └── test/ # 测试代码
└── README.md # 项目说明文件
## 说明

- `solver/include/`：放置所有头文件（如类声明、接口定义等）。
- `solver/src/`：放置主要的源代码实现。
- `solver/test/`：用于单元测试、验证算法正确性。
- `.vs/`：Visual Studio 自动生成的工程配置文件，可以不用关心。


---

## Features

- Fully C++ implementation with extensible design
- Solves quadratic programming (QP) and CCP problems in multibody simulation
- Ready to be integrated into larger simulation frameworks
- Modular, easy to maintain and extend
- Compatible with GitHub Codespaces or local development

---

## Typical Application

- Simulation of robotics, vehicle systems, mechanical linkages, and contact/impact modeling
- Research in numerical optimization for physics engines

---

## Quick Start

```bash
# Clone this repository
git clone https://github.com/Invincible-ZHANG/my_apgd_project.git

# Build (example for CMake-based project)
cd my_apgd_project
mkdir build && cd build
cmake ..
make



