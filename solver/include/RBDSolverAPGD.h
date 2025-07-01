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

        virtual Type GetType() const override { return Type::APGD; }

        /// Performs the solution of the problem.
        virtual double Solve(RBDSystemDescriptor& sysd) override;

        /// Return the tolerance error reached during the last solve.
        /// For the APGD solver, this is the norm of the projected gradient.
        virtual double GetError() const override { return residual; }

        void Dump_Rhs(std::vector<double>& temp);
        void Dump_Lambda(std::vector<double>& temp);

    private:
        void SchurBvectorCompute(ChSystemDescriptor& sysd);
        double Res4(ChSystemDescriptor& sysd);

        double residual;
        int nc;
        VSM::VectorNDynamic gamma_hat, gammaNew, g, y, gamma, yNew, r, tmp;
    };

    /// @} VSLibRBDynamX_solver

}  // end namespace VSLibRBDynamX

#endif



