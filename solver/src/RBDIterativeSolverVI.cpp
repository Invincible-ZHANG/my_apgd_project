//// =============================================================================
//// VSLibRBDynamX – RBDIterativeSolverVI Implementation
////
//// RBDIterativeSolverVI.cpp
////   Implementation file for the base class of iterative solvers designed to solve
////   variational inequality (VI) and complementarity problems in multibody dynamics.
////
////   The RBDIterativeSolverVI class provides:
////     - Core interfaces for iterative CCP/LCP/QP solvers
////     - Support for setting relaxation/sharpness factors, maximum iterations, 
////       and violation history recording
////     - Serialization (archive in/out) for solver settings and states
////
////   Typical use cases: Gauss-Seidel, APGD, SOR, Projected Gradient, etc.
////
//// Copyright (c) 2025 Zijian Zhang
//// All rights reserved.
////
//// =============================================================================
//// Authors: Zijian Zhang
//// Date:    2025-07-02
//// =============================================================================
//
//
//#include "RBDIterativeSolverVI.h"
//
//namespace VSLibRBDynamX {
//    
//    //CH_UPCASTING(ChIterativeSolverVI, ChIterativeSolver)
//    //CH_UPCASTING(ChIterativeSolverVI, ChSolverVI)
//    //CH_UPCASTING(ChSolverVI, ChSolver)  // placed here since ChSolver is missing the .cpp
//
//    RBDIterativeSolverVI::RBDIterativeSolverVI()
//        : RBDIterativeSolver(50, 0.0, true, false),
//        m_omega(1.0),
//        m_shlambda(1.0),
//        m_iterations(0),
//        record_violation_history(false) {}
//
//    void RBDIterativeSolverVI::SetOmega(double mval) {
//        if (mval > 0.)
//            m_omega = mval;
//    }
//
//    void RBDIterativeSolverVI::SetSharpnessLambda(double mval) {
//        if (mval > 0.)
//            m_shlambda = mval;
//    }
//
//    void RBDIterativeSolverVI::SetMaxIterations(int max_iterations) {
//        m_max_iterations = max_iterations;
//        violation_history.resize(m_max_iterations);
//        dlambda_history.resize(m_max_iterations);
//    }
//
//    void RBDIterativeSolverVI::SetRecordViolation(bool mval) {
//        record_violation_history = mval;
//        SetMaxIterations(m_max_iterations);
//    }
//
//    void RBDIterativeSolverVI::AtIterationEnd(double mmaxviolation, double mdeltalambda, unsigned int iternum) {
//        if (!record_violation_history)
//            return;
//
//        if (iternum >= violation_history.size()) {
//            assert(false && "Try to access out-of-bound.");
//            return;
//        }
//
//        violation_history[iternum] = mmaxviolation;
//        dlambda_history[iternum] = mdeltalambda;
//    }
//
//    void RBDIterativeSolverVI::ArchiveOut(ChArchiveOut& archive_out) {
//        // version number
//        archive_out.VersionWrite<RBDIterativeSolverVI>();
//        // serialize parent class
//        RBDSolver::ArchiveOut(archive_out);
//        // serialize all member data:
//        archive_out << CHNVP(m_max_iterations);
//        archive_out << CHNVP(m_warm_start);
//        archive_out << CHNVP(m_tolerance);
//        archive_out << CHNVP(m_omega);
//        archive_out << CHNVP(m_shlambda);
//    }
//
//    void RBDIterativeSolverVI::ArchiveIn(ChArchiveIn& archive_in) {
//        // version number
//        /*int version =*/archive_in.VersionRead<ChIterativeSolverVI>();
//        // deserialize parent class
//        RBDSolver::ArchiveIn(archive_in);
//        // stream in all member data:
//        archive_in >> CHNVP(m_max_iterations);
//        archive_in >> CHNVP(m_warm_start);
//        archive_in >> CHNVP(m_tolerance);
//        archive_in >> CHNVP(m_omega);
//        archive_in >> CHNVP(m_shlambda);
//    }
//
//}  // end namespace VSLibRBDynamX
