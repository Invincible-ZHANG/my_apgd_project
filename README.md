# My_APGD_project

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

## 软件架构

**Idea** :Chrono 的求解器结构确实更清晰、模块化和易扩展，而 VEROSIM（VSLibRBDynamX）目前是传统的“一锅粥”式组装风格。我想在 VEROSIM 实现类似 Chrono 架构的 APGD 求解器，并结合多体动力学的 QP/CCP 实际需求。

### 1. 设计目标
- 解耦数据结构和算法实现：约束、变量、系统描述、求解器分开。
- 接口统一、可扩展性强：易于插拔和后续拓展。
- 最大限度参考 Chrono 结构：如 ChSystemDescriptor / ChVariables / ChConstraint / ChSolver。
- 兼容 VEROSIM 原有数据结构：保留你的 RBDScene、RBDRigidBody、RBDConstraintResource 等，但提供一套新的 solver 管理接口。 

### 2. 主要类与接口设计

#### （1）系统描述器 RBDSystemDescriptor
类似 Chrono 的 ChSystemDescriptor，负责统一管理整个动力学系统的变量与约束集合，并提供组装/分派接口。

```
class RBDSystemDescriptor {
public:
    // 变量和约束集合
    std::vector<RBDVariables*> variables;
    std::vector<RBDConstraint*> constraints;

    // 添加、移除
    void AddVariable(RBDVariables* v);
    void AddConstraint(RBDConstraint* c);

    // 分别将全局向量分发给子对象、或从子对象收集
    void FromVariablesToVector(VectorXd& x);
    void FromVectorToVariables(const VectorXd& x);

    void FromConstraintsToVector(VectorXd& l);
    void FromVectorToConstraints(const VectorXd& l);

    // Schur 补/投影/摩擦锥/残差等接口
    void SchurComplementProduct(VectorXd& result, const VectorXd& lambda);
    void ConstraintsProject(VectorXd& lambda);
    void BuildBiVector(VectorXd& b);

    // ...
};

```

#### （2）变量类 RBDVariables
每个刚体（或广义坐标块），抽象成变量对象，负责自己的质量矩阵、力、状态更新。

```
class RBDVariables {
public:
    virtual void ComputeMassInverseTimesVector(VectorXd& result, const VectorXd& f) = 0;
    virtual void SetState(const VectorXd& x) = 0;
    virtual void GetState(VectorXd& x) = 0;
    // ...
};

```

#### （3）约束类 RBDConstraint
每个约束类型（比如接触、关节等），实现标准接口。

```
class RBDConstraint {
public:
    virtual void Update_auxiliary() = 0; // 更新临时数据，如 Jacobian
    virtual void ComputeJacobianTimesState(VectorXd& result, const VectorXd& x) = 0;
    virtual void Project(VectorXd& lambda) = 0;
    // 约束类型（等式、LCP、CCP等）也可以由成员变量区分
};

```

#### （4）求解器基类 RBDIterativeSolverVI
与 Chrono 的 ChIterativeSolverVI 类似，所有求解器（APGD/PSOR/PGS等）派生于此。

```
class RBDIterativeSolverVI {
public:
    virtual double Solve(RBDSystemDescriptor& sysd) = 0;
    virtual double GetError() const = 0;
    // 通用接口：最大迭代数、收敛容忍度等
};

```

#### （5）APGD 求解器 RBDSolverAPGD
实现核心算法，依赖上面所有接口而不是直接访问原始物理对象。

```
class RBDSolverAPGD : public RBDIterativeSolverVI {
public:
    double Solve(RBDSystemDescriptor& sysd) override;
    double GetError() const override;
    // APGD 的状态变量等
};
```

### 3. 软件架构/调用流程
```
RBDScene
   │
   ├─ 封装所有 RBDRigidBody/RBDConstraintResource
   │
   ├─ 初始化 RBDSystemDescriptor
   │        │
   │        ├─ AddVariable(RBDRigidBody → RBDVariables)
   │        └─ AddConstraint(RBDConstraintResource → RBDConstraint)
   │
   └─ 调用 Solver（RBDSolverAPGD::Solve(sysd)）
              │
              ├─ 用接口组装 Schur 补/系统矩阵和向量
              └─ 完成主循环、迭代等
```
所有的组装和矩阵-向量操作全部通过接口，不直接操作物理对象本身，这样以后换求解器、调试子模块都更容易。

### 4. 前期迁移步骤
- 抽象接口：优先把 VEROSIM 现有刚体/约束数据，抽象成 RBDVariables 和 RBDConstraint 接口实现。
- 实现 RBDSystemDescriptor：统一管理变量和约束集合，实现类似 Chrono 的分发和组装接口。
- 实现 APGD 算法类，只依赖 RBDSystemDescriptor，不直接操作原有物理对象。
- 测试用例：先用纯等式约束/最小系统测试，再逐步扩展到不等式、摩擦、更多刚体。


### 5. 后期展望工作
- 后期可扩展：如果后续需要支持并行、SIMD、GPU，可以方便地在接口内加实现，无需改动主结构。
- 易于调试和可视化：接口分明，便于插拔历史记录/约束检查/收敛曲线等工具。
- 借鉴 Chrono 的代码风格和命名，未来迁移或集成外部 solver 都很方便。

### 6. 架构示意图

```
[ RBDScene ]
     │
[ RBDSystemDescriptor ]
     ├────[ RBDVariables ] ← RBDRigidBody
     └────[ RBDConstraint ] ← RBDConstraintResource/RBDContact/...
     │
[ RBDSolverAPGD ]
     │
<—全部通过接口交互、无耦合—>
```

### 7. 示例代码调用

```
// 初始化
RBDSystemDescriptor sysd;
for (auto* body : allBodies)
    sysd.AddVariable(new MyRBDVariables(body));
for (auto* constr : allConstraints)
    sysd.AddConstraint(new MyRBDConstraint(constr));

// 求解
RBDSolverAPGD apgd_solver;
apgd_solver.Solve(sysd);
```


---
## 文件结构

```
my_apgd_project/
├── .vs/ # Visual Studio 工程文件夹
├── solver/ # 源码主目录
│ ├── include/ # 头文件（接口、声明等）
│ ├── src/ # 源文件（主要实现）
│ └── test/ # 测试代码
└── README.md # 项目说明文件
```

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

```

## 更新日志 (Changelog)

---
### 2025-07-04

 - 完善Adapter/Wrapper 类
 - 脱离VEROSIM做一个简单DEMO测试工作
  - 不依靠chrono和verosim的任何相关库来完成这个demo。

---
### 2025-07-03

 - 增加了接口文件：RBDInterface

---
### 2025-07-02

- **add RBDIterativeSolverVI.cpp and RBDIterativeSolverVI.h**  
  新增迭代型 VI 求解器的头文件与实现。

- **add APGD SOLVER COMMENT**  
  补充 APGD 求解器相关代码注释，提升可读性。

---

### 2025-07-01

- **modified APGD solver related cpp and h file**  
  修改 APGD 求解器相关实现，修复细节，提升稳定性。

- **README Update**  
  更新项目说明文档。

- **add VS project file, RBDSolverAPGD.cpp and RBDSolverAPGD.h**  
  新增 Visual Studio 工程文件，补充 APGD 求解器主文件。

- **Initial commit**  
  项目初始化，创建基本文件结构。



## 实施分阶段计划

### 阶段一：接口抽象和数据映射
1. 定义基类/接口文件（新建文件夹如 RBDInterface）
   - RBDSystemDescriptor：统一管理变量、约束对象的接口。
   - RBDVariables、RBDConstraint：抽象刚体、约束数据和操作。

2. 为原有 RBDRigidBody/RBDConstraintResource 写 Adapter/Wrapper 类

    - 例如 class MyRBDVariables : public RBDVariables，内部保存指针指向 RBDRigidBody。

    - 在函数如 ComputeMassInverseTimesVector 内部，直接调用 RBDRigidBody 的数据成员和方法。

    - 这样新 solver 只用面向接口，物理数据“通过一层壳”访问。

3. 开发和测试 SystemDescriptor 基本操作

    - 能够正确将所有 bodies/constraints 添加进集合，并实现变量与全局向量的读写转换。

    - 先写最基础的功能，比如收集状态量、批量更新。


### 阶段二：核心求解器迁移与接口实现
1. 实现 RBDSolverAPGD

   - 按照 Chrono 的结构写 APGD 主循环，完全依赖接口，严禁直接访问 RBDRigidBody/RBDConstraintResource。

   - 参数、状态量都为局部变量或通过接口维护。

2. 实现所有接口必要函数

   - 比如 Schur 补的组装、Jacobian 投影、变量分配、残差计算、ConstraintsProject 等。

   - 可以参考 Chrono 的 ChSystemDescriptor / ChConstraint 等具体实现。

3. 单元测试（最小系统）

   - 用极简的刚体、单一等式/不等式约束，确保“端到端”流程无误。


### 阶段三：集成、调试和性能优化
1. 将新 APGD 求解器嵌入主仿真流程

   - 修改物理主循环，将原有的组装和解法部分替换为新接口和新 solver 的调用。

   - 关键是“插拔式”：允许随时切换老 solver（如 LCP GS/Dantzig）与新 APGD，对比效果。

2. 添加日志、调试接口

   - 每步记录约束残差、lambda 变化等，方便画收敛曲线。

   - 可以扩展 DebugObserver，将迭代历史输出到文件。

3. 逐步收敛所有新模块为接口化调用

   - 逐步清理旧的冗余代码，只保留面向接口的新结构。

4. 优化与扩展

   - 支持多线程/SIMD（如果有需求）。

   - 支持多种约束类型、摩擦锥、软约束等扩展。


## Q&A:
### Chrono vs. Verosim 矩阵组装机制怎么理解？
**Q1: 我能不能这样理解——Chrono 是“只装需要的小块”，Verosim 是“全量填大表”？Chrono 更适合大规模物体？**


A1: 完全可以这么理解。

Chrono 的做法
 - 每个约束、刚体只管理自身相关的那部分数据，临时组装，无关的自由度就不管/不存（在全局矩阵里“为0”，但实现时根本不分配也不操作）。
 - 计算时才临时拼起来，极度稀疏、灵活，没有全局大矩阵的负担。
- 适合大规模、多物体系统，计算效率高，内存友好。

Verosim 的做法
- 会为所有刚体、所有自由度分配一个全局大矩阵，每个约束、每个物体都被静态映射到某一行某一列。
- 哪怕绝大多数都是0，结构上也要全部分配好（实现时可能是稀疏存储，但逻辑上是“大表”）。
 便于一目了然查看、调试、输出，适合教学、小规模场景。

**总结:**
- Chrono 适合大系统（大场景，万级物体），因为它节省内存，只操作需要的块，高效。

- Verosim 适合小型或教学用途，结构清楚，便于理解和可视化。


### Adapter/Wrapper

**Q2： 什么是Adapter/Wrapper这一套做法到底是干什么用的？**

A2 ：类比：
  - 假如你有很多种不同品牌、不同类型的插头（物理对象），
   -但是你只想用同一个插座（Solver），
   -你就需要一个转换头（Adapter/Wrapper），这样无论插头什么样，只要有转换头都能用同一个插座。
对应到你的工程：
 -物理对象（比如 RBDRigidBody、RBDConstraintResource）：每个数据结构不同，函数不同。
 -Solver（求解器）：需要“拿到数据、做统一的操作”（比如计算质量逆、设置状态等）。
 -RBDVariables（接口）：规定了一套“插头标准”（有 GetDOF、GetState、SetState、ComputeMassInverseTimesVector 这些统一操作）。
 -Adapter/Wrapper（MyRBDVariables）：就是转换头，让各种物理对象都能装到 Solver 里统一用。

实际作用/优点：
 -你的Solver 只依赖接口，不依赖具体实现，即“只认插头标准，不认品牌”。
 -以后你增加新的物理对象（比如柔体、流体、某种奇怪的刚体），不用改 Solver，只用写新的 Wrapper 就能接入。
 -代码扩展性、可维护性极强，比如 Chrono、PhysX、Bullet 等物理引擎底层都是这么做的。



