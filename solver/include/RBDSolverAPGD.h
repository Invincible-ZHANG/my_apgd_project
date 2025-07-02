// =============================================================================
// VSLibRBDynamX – Accelerated Projected Gradient Descent Solver
//
// RBDSolverAPGD.h
//   Iterative solver implementation based on Nesterov's Accelerated Projected
//   Gradient Descent (APGD) for solving variational inequality (VI) problems
//   arising in multibody dynamics with complementarity constraints.
//
//   The APGD algorithm is efficient for large-scale CCP (Cone Complementarity Problem)
//   systems and supports projected gradient minimization under convex constraints.
//
// Copyright (c) 2025 Zijian Zhang
// All rights reserved.
//
// =============================================================================
// Authors: Zijian Zhang
// Date:    2025-07-01
// =============================================================================

#ifndef CLASS_RBDYNAMX_RBDSOLVERAPGD
#define CLASS_RBDYNAMX_RBDSOLVERAPGD

#include "RBDIterativeSolverVI.h"

namespace VSLibRBDynamX {

/// @addtogroup VSLibRBDynamX_solver
/// @{

/// An iterative solver based on Nesterov's Projected Gradient Descent.
///
/// See ChSystemDescriptor for more information about the problem formulation and the data structures passed to the
/// solver.
    class RBDSolverAPGD : public RBDIterativeSolverVI {
    public:
        RBDSolverAPGD();

        ~RBDSolverAPGD() {}

        /// 返回该求解器的类型（APGD）。用于工厂注册或多态类型识别。
        virtual Type GetType() const override { return Type::APGD; }

        /// Performs the solution of the problem.
        /// 核心主函数，执行APGD算法，求解当前系统的VI问题。
        virtual double Solve(RBDSystemDescriptor& sysd) override;

        /// Return the tolerance error reached during the last solve.
        /// For the APGD solver, this is the norm of the projected gradient.
        /// 返回本次求解过程的最终误差（通常为投影梯度的范数）。
        virtual double GetError() const override { return residual; }

        /// 将右端项（b向量、残差等）导出到外部vector中。
        void Dump_Rhs(std::vector<double>& temp);

        /// 导出求解得到的拉格朗日乘子（lambda），即最终的对偶变量。
        void Dump_Lambda(std::vector<double>& temp);

    private:
        /// 生成APGD算法中的Schur补右端向量。
        void SchurBvectorCompute(ChSystemDescriptor& sysd);

        /// 计算当前解的投影梯度范数，即收敛残差。
        double Res4(ChSystemDescriptor& sysd);
        /*
        ===================================================================
        APGD Solver State Variables（APGD求解器核心状态变量说明）

        这些成员变量用于存储APGD（加速投影梯度下降）算法在多体动力学补体约束问题
        迭代过程中的全部关键向量、误差和迭代控制信息。主要包括：

        - residual：每次迭代收敛误差（即投影梯度的范数），用于终止条件判断。
        - nc      ：当前有效约束/变量数量（维数）。
        - gamma/gammaNew/gamma_hat：拉格朗日乘子向量，分别表示当前解、下一步解、最优历史解。
        - y/yNew  ：Nesterov加速辅助变量（搜索点）。
        - g       ：当前梯度向量（用于方向更新）。
        - r       ：Schur补右端项向量（APGD主方程右侧）。
        - tmp     ：通用临时向量缓冲区，用于中间步骤计算。

        这些变量的维数动态分配，依据实际约束数量自动适配。
        算法每步迭代会不断更新这些变量，最终收敛到满足KKT条件的最优解。

        --------------------------------------------------------------
        These variables store all core APGD (Accelerated Projected Gradient Descent)
        solver state during the iterative solution of the cone complementarity
        (CCP) problem in multibody dynamics.

        - residual:    Norm of projected gradient, used as convergence criterion.
        - nc:          Number of active constraints (problem size).
        - gamma/gammaNew/gamma_hat: Lagrange multiplier vectors for current, next, and best (lowest residual) states.
        - y/yNew:      Nesterov acceleration auxiliary variables (search points).
        - g:           Current gradient vector for direction update.
        - r:           Schur complement right-hand-side vector.
        - tmp:         General-purpose temporary buffer for intermediate calculations.

        All vectors are dynamically resized to match the current problem size.
        The APGD iteration updates these variables step-by-step until convergence.
        ===================================================================
    */

        double residual;
        int nc;
        VSM::VectorNDynamic gamma_hat, gammaNew, g, y, gamma, yNew, r, tmp;
    };

    /// @} VSLibRBDynamX_solver

}  // end namespace VSLibRBDynamX

#endif



