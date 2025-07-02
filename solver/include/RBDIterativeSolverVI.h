// =============================================================================
// VSLibRBDynamX – Iterative Variational Inequality Solver Base Class
//
// RBDIterativeSolverVI.h
//    本文件定义了迭代型补体问题求解器的抽象基类，用于实现多体动力学仿真中的
//    各类变分不等式（VI）、互补约束（CCP/LCP）、二次规划（QP）问题的数值解法。
//    提供了主流PSOR、Jacobi、APGD等算法的通用接口、参数配置、收敛历史管理等功能。
//    支持不同类型的可行域投影（实数域、正交锥、摩擦锥），为派生具体求解器奠定框架。
//
//    This header defines the abstract base class for all iterative solvers
//    targeting complementarity problems (VI/LCP/CCP) arising from QP formulations
//    in multibody dynamics. It provides common algorithm parameters, iteration
//    management, and interface for recording constraint violation histories.
//    The class enables the implementation of various iterative methods
//    (e.g., PSOR, Jacobi, APGD) with consistent configuration and statistics.
// 
// Copyright (c) 2025 Zijian Zhang
// All rights reserved.
//
// =============================================================================
// Authors: Zijian Zhang
// Date:    2025-07-02
// =============================================================================

#ifndef CLASS_RBDYNAMX_RBDITERATIVESOLVERVI
#define CLASS_RBDYNAMX_RBDITERATIVESOLVERVI

#include "RBDSolverVI.h"
#include "RBDIterativeSolver.h"

namespace VSLibRBDynamX {

    /// @addtogroup VSLibRBDynamX_solver
    /// @{

    /** \class RBDIterativeSolverVI
    \brief Base class for iterative solvers aimed at solving complementarity problems arising from QP optimization problems.

    The problem is described by a variational inequality VI(Z*x-d,K):\n

    <pre>
      | M -Cq'|*|q|- | f|= |0|
      | Cq -E | |l|  |-b|  |c|
    </pre>
    with l \f$\in\f$ Y, C \f$\in\f$ Ny, normal cone to Y\n

    Also Z symmetric by flipping sign of l_i:
    <pre>
      |M  Cq'|*| q|-| f|=|0|
      |Cq  E | |-l| |-b| |c|
    </pre>

    - case linear problem:  all Y_i = R, Ny=0, ex. all bilaterals
    - case LCP: all Y_i = R+:  c>=0, l>=0, l*c=0
    - case CCP: Y_i are friction cones

    The default maximum number of iterations is 50.

    The 'tolerance' threshold value used in the stopping criteria may have different meaning for different solvers
    (threshold on maximum constraint violation, threshold on projected residual, etc). See the GetError method for an
    individual iterative VI solver for details.

    Diagonal preconditioning is enabled by default, but may not supported by all iterative VI solvers.
    */
    class RBDIterativeSolverVI : public RBDIterativeSolver, public RBDSolverVI {
    public:
        RBDIterativeSolverVI();

        virtual ~RBDIterativeSolverVI() {}

        /// Set the overrelaxation factor (default: 1.0).
        /// This factor may be used by PSOR-like methods. A good value for Jacobi solver is 0.2; for other iterative solvers
        /// it can be up to 1.0
        void SetOmega(double mval);

        /// Set the sharpness factor (default: 1.0).
        /// This factor may be used by PSOR-like methods with projection (see Mangasarian LCP method). A good sharpness
        /// value is in the 0.8 ... 1.0 range (lower values improve accuracy but at the cost of slower convergence)
        void SetSharpnessLambda(double mval);

        /// Set the maximum number of iterations.
        virtual void SetMaxIterations(int max_iterations) override;

        /// Enable/disable recording of the constraint violation history.
        /// If enabled, the maximum constraint violation at the end of each iteration is stored in a vector (see
        /// GetViolationHistory).
        void SetRecordViolation(bool mval);

        /// Return the current value of the overrelaxation factor.
        double GetOmega() const { return m_omega; }

        /// Return the current value of the sharpness factor.
        double GetSharpnessLambda() const { return m_shlambda; }

        /// Return the number of iterations performed during the last solve.
        virtual int GetIterations() const override { return m_iterations; }

        /// Access the vector of constraint violation history.
        /// Note that collection of constraint violations must be enabled through SetRecordViolation.
        const std::vector<double>& GetViolationHistory() const { return violation_history; }

        /// Access the vector with history of maximum change in Lagrange multipliers
        /// Note that collection of constraint violations must be enabled through SetRecordViolation.
        const std::vector<double>& GetDeltalambdaHistory() const { return dlambda_history; }

        /// Method to allow serialization of transient data to archives.
        virtual void ArchiveOut(ChArchiveOut& archive_out) override;

        /// Method to allow de-serialization of transient data from archives.
        virtual void ArchiveIn(ChArchiveIn& archive_in) override;

    protected:
        virtual bool IsIterative() const override { return true; }
       
        /// 返回false，声明本类不是直接解法（Direct Solver），而是迭代型。
        virtual bool IsDirect() const override { return false; }
        virtual RBDIterativeSolver* AsIterative() override { return this; }

        /// This method MUST be called by all iterative methods INSIDE their iteration loops
        /// (at the end). If history recording is enabled, this function will store the
        /// current values as passed as arguments.
        /// Note: 'iternum' starts at 0 for the first iteration.
        /// 仅供派生类在每轮迭代结尾处调用，用于自动存储该轮的收敛信息（约束违背量、乘子变化量、轮数）。
        void AtIterationEnd(double mmaxviolation, double mdeltalambda, unsigned int iternum);

    protected:
        /// Indicate whether ot not the Solve() phase requires an up-to-date problem matrix.
        /// Typically, this is the case for iterative solvers (as the matrix is needed for
        /// the matrix-vector operations).
        /// 表示当前求解器（本类及其派生类）在每次 Solve() 调用时，是否需要系统最新构建的“问题矩阵”（比如雅可比矩阵、质量矩阵或 Schur 补矩阵等）。
        virtual bool SolveRequiresMatrix() const override { return true; }

        int m_iterations;   ///< number of iterations performed by the solver during last call to Solve()
        double m_omega;     ///< over-relaxation factor
        double m_shlambda;  ///< sharpness factor

        bool record_violation_history;
        std::vector<double> violation_history;
        std::vector<double> dlambda_history;
    };

    /// @} VSLibRBDynamX_solver

}  // end namespace VSLibRBDynamX

#endif
