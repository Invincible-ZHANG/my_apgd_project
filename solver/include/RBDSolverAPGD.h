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

#pragma once

#include "RBDIterativeSolverVI.h"
#include "RBDSystemDescriptor.h"
#include <vector>

namespace VSLibRBDynamX {

    /// @addtogroup VSLibRBDynamX_solver
    /// @{

    /// An iterative solver based on Nesterov's Projected Gradient Descent.
    ///
    /// The APGD algorithm is efficient for large-scale CCP systems and supports
    /// projected gradient minimization under convex constraints.
    class RBDSolverAPGD : public RBDIterativeSolverVI {
    public:
        /// Constructor
        RBDSolverAPGD();

        /// Destructor
        ~RBDSolverAPGD() = default;

        /// Performs the solution of the problem.
        /// 核心函数，执行 APGD 算法，求解系统 VI 问题。
        double Solve(RBDSystemDescriptor& sysd);

        /// Return the tolerance error reached during the last solve.
        /// 对于 APGD 求解器，这是投影梯度的范数。
        double GetError() const { return residual; }

        /// 导出右端项向量 r
        void Dump_Rhs(std::vector<double>& temp) const { temp = r; }

        /// 导出最终的拉格朗日乘子向量 lambda
        void Dump_Lambda(std::vector<double>& temp) const { temp = gamma; }

    private:
        /// 生成 APGD 算法中的 Schur 补右端向量 r
        void SchurBvectorCompute(RBDSystemDescriptor& sysd);
        int m_iterations;    ///< 当前迭代轮数

        /// 计算当前解的投影梯度范数，作为收敛残差
        double Res4(const std::vector<double>& lambda) const;

        double residual;                 ///< 当前迭代收敛误差
        int nc;                          ///< 问题维数 (约束数)
        std::vector<double> gamma;       ///< 当前拉格朗日乘子
        std::vector<double> gammaNew;    ///< 下一步拉格朗日乘子
        std::vector<double> gamma_hat;   ///< 历史最佳解
        std::vector<double> y;           ///< Nesterov 加速辅助变量
        std::vector<double> yNew;        ///< 下一步辅助变量
        std::vector<double> g;           ///< 当前梯度
        std::vector<double> r;           ///< Schur 补右端向量
        std::vector<double> tmp;         ///< 中间临时缓冲区
    };

    /// @} VSLibRBDynamX_solver

} // namespace VSLibRBDynamX
