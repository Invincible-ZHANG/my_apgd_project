#ifndef CLASS_RBDYNAMX_RBDITERATIVESOLVERVI
#define CLASS_RBDYNAMX_RBDITERATIVESOLVERVI

// =============================================================================
// VSLibRBDynamX – Iterative Solver Base Class for APGD
//
// RBDIterativeSolverVI.h
//   精简后的迭代求解器基类，仅保留 APGD 算法所需接口：
//   - Solve 接口
//   - 迭代次数管理
//   - 收敛历史记录
//   - Over-relaxation 与 Sharpness 参数
//   - AtIterationEnd 用于记录每轮残差与乘子变化
//
// Copyright (c) 2025 Zijian Zhang
// All rights reserved.
//
// Authors: Zijian Zhang
// Date:    2025-07-02
// =============================================================================

#pragma once

#include <vector>
#include "RBDSystemDescriptor.h"

namespace VSLibRBDynamX {

    /// 精简版迭代求解器基类，仅支持 APGD 相关操作。
    class RBDIterativeSolverVI {
    public:
        virtual ~RBDIterativeSolverVI() {}

        /// 主求解接口，派生类必须实现
        virtual double Solve(RBDSystemDescriptor& sysd) = 0;

        /// 返回本次求解的误差量（通常为投影梯度范数）
        virtual double GetError() const = 0;

        /// 设置最大迭代轮数
        void SetMaxIterations(int max_iter) { m_max_iterations = max_iter; }
        int GetMaxIterations() const { return m_max_iterations; }

        /// 设置收敛阈值
        void SetTolerance(double tol) { m_tolerance = tol; }
        double GetTolerance() const { return m_tolerance; }

        /// 设置 Over-relaxation 因子 ω（一般 ≤1.0）
        void SetOmega(double w) { m_omega = w; }
        double GetOmega() const { return m_omega; }

        /// 设置 Sharpness 因子 λ（一般 ≤1.0）
        void SetSharpnessLambda(double s) { m_shlambda = s; }
        double GetSharpnessLambda() const { return m_shlambda; }

        /// 是否记录每轮违背历史
        void SetRecordViolation(bool record) {
            record_violation = record;
            violation_history.clear();
            dlambda_history.clear();
        }
        const std::vector<double>& GetViolationHistory() const { return violation_history; }
        const std::vector<double>& GetDeltalambdaHistory() const { return dlambda_history; }

    protected:
        RBDIterativeSolverVI()
            : m_max_iterations(1000), m_tolerance(1e-6), m_omega(1.0), m_shlambda(1.0), record_violation(false) {}

        /// 迭代结束时调用，自动记录残差与乘子变化
        void AtIterationEnd(double max_violation, double delta_lambda, unsigned int iter) {
            if (!record_violation) return;
            if (iter >= violation_history.size()) {
                violation_history.resize(m_max_iterations);
                dlambda_history.resize(m_max_iterations);
            }
            violation_history[iter] = max_violation;
            dlambda_history[iter] = delta_lambda;
        }

        int m_max_iterations;            ///< 最大迭代轮数
        double m_tolerance;              ///< 收敛阈值
        double m_omega;                  ///< Over-relaxation 因子
        double m_shlambda;               ///< Sharpness 因子

        bool record_violation;           ///< 是否记录迭代历史
        std::vector<double> violation_history;
        std::vector<double> dlambda_history;
    };

}  // namespace VSLibRBDynamX

#endif // CLASS_RBDYNAMX_RBDITERATIVESOLVERVI
