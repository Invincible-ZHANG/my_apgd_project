
#include <iomanip>
#include "RBDIterativeSolver.h"

namespace VSLibRBDynamX {

    RBDIterativeSolver::RBDIterativeSolver(int max_iterations, double tolerance, bool use_precond, bool warm_start)
        : m_max_iterations(max_iterations), m_tolerance(tolerance), m_use_precond(use_precond), m_warm_start(warm_start) {}

    void RBDIterativeSolver::WriteMatrices(RBDSystemDescriptor& sysd, bool one_indexed) {
        // Assemble sparse matrix
        RBDSparseMatrix Z1;
        sysd.BuildSystemMatrix(&Z1, nullptr);

        // Create sparse matrix with SPMV
        RBDMatrixDynamic<> Z2(Z1.rows(), Z1.cols());
        RBDVectorDynamic<> e = RBDVectorDynamic<>::Zero(Z1.cols());
        RBDVectorDynamic<> v(Z1.cols());
        for (int i = 0; i < Z1.cols(); i++) {
            e(i) = 1;
            sysd.SystemProduct(v, e);
            Z2.col(i) = v;
            e(i) = 0;
        }

        // Save matrices to file
        {
            std::ofstream file("Z1.dat");
            file << std::setprecision(12) << std::scientific;
            StreamOut(Z1, file, one_indexed);
        }
        {
            std::ofstream file("Z2.dat");
            file << std::setprecision(12) << std::scientific;
            StreamOut(Z2, file);
        }

        // Assemble RHS
        RBDVectorDynamic<> rhs1;
        sysd.BuildSystemMatrix(nullptr, &rhs1);

        // RHS using d vector
        RBDVectorDynamic<> rhs2;
        sysd.BuildDiVector(rhs2);

        // Save vectors to file
        {
            std::ofstream file("rhs1.dat");
            file << std::setprecision(12) << std::scientific;
            StreamOut(rhs1, file);
        }
        {
            std::ofstream file("rhs2.dat");
            file << std::setprecision(12) << std::scientific;
            StreamOut(rhs2, file);
        }
    }

    double RBDIterativeSolver::CheckSolution(RBDSystemDescriptor& sysd, const RBDVectorDynamic<>& x) {
        RBDVectorDynamic<> b;
        sysd.BuildSystemMatrix(nullptr, &b);

        RBDSparseMatrix Z;
        sysd.BuildSystemMatrix(&Z, nullptr);
        double res_norm1 = (Z * x - b).norm();

        RBDVectorDynamic<> Zx(x.size());
        sysd.SystemProduct(Zx, x);
        double res_norm2 = (Zx - b).norm();

        std::cout << "  Residual norm (using full matrix): " << res_norm1 << std::endl;
        std::cout << "  Residual norm (using SPMV):        " << res_norm2 << std::endl;

        return res_norm1;
    }

}  // end namespace VSLibRBDynamX
