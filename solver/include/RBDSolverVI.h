

#ifndef CLASS_RBDYNAMX_RBDSOLVERVI
#define CLASS_RBDYNAMX_RBDSOLVERVI

#include "RBDSolver.h"

namespace VSLibRBDynamX {

    /// @addtogroup VSLibRBDynamX_solver
    /// @{

    /** \class RBDSolverVI
    \brief Base class for solvers aimed at solving complementarity problems arising from QP optimization problems.

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

    For details on the supported types of solvers see ChIterativeSolverVI and the concrete iterative VI solvers.
    */
    class RBDSolverVI : public RBDSolver {
    public:
        virtual ~RBDSolverVI() {}
    };

    /// @} VSLibRBDynamX_solver

}  // end namespace VSLibRBDynamX

#endif