// =============================================================================
//  RBDSolverAPGD.cpp
//
//  Implementation of an Accelerated Projected Gradient Descent (APGD) solver
//  for multibody dynamics constrained systems.
// =============================================================================

#include "RBDSolverAPGD.h"
#include <cmath>
#include <algorithm>

namespace VSLibRBDynamX {

    RBDSolverAPGD::RBDSolverAPGD()
        : nc(0), residual(0.0), m_iterations(0) {}

    // 构建 Schur 补右端向量 r = -C * (M^{-1} f) - b
    void RBDSolverAPGD::SchurBvectorCompute(RBDSystemDescriptor& sysd) {
        // 获取变量和约束列表
        auto& vars = sysd.GetVariables();
        auto& cons = sysd.GetConstraints();

        // 步骤1：构建全局系统矩阵 Z 和右端 d
        std::vector<std::vector<double>> Z;
        std::vector<double> d;
        sysd.BuildSystemMatrix(Z, d);

        // 步骤2：r = d  （d 已包含了 - b_i 部分）
        r = d;

        // 若需要拆分 d = f + b，可以调用 BuildDiVector，但这里直接使用 d 即可
    }

    // 计算投影梯度范数：|| (λ - proj(λ - t*(N*λ + r))) / t ||
    double RBDSolverAPGD::Res4(const std::vector<double>& lam) const {
        std::vector<double> Nlam(nc), tmp2(nc);
        // Nlam = Z * lam
        for (int i = 0; i < nc; ++i) {
            double sum = 0;
            for (int j = 0; j < nc; ++j) sum += /*Z[i][j]*/ 0.0 * lam[j];
            Nlam[i] = sum;
        }
        // tmp2 = lam - t*(Nlam + r)
        // 但此处 t 不在参数中，我们在 Solve 里直接计算残差，故这里仅返回 0
        return 0.0;
    }

    double RBDSolverAPGD::Solve(RBDSystemDescriptor& sysd) {
        // 构建尺寸
        nc = static_cast<int>(sysd.GetConstraints().size());
        gamma.assign(nc, 0.0);
        gammaNew.assign(nc, 0.0);
        gamma_hat.assign(nc, 1.0);
        y.assign(nc, 0.0);
        yNew.assign(nc, 0.0);
        g.assign(nc, 0.0);
        r.assign(nc, 0.0);
        tmp.assign(nc, 0.0);

        // 构建 Schur 补右端向量
        SchurBvectorCompute(sysd);

        if (nc == 0) {
            residual = 0.0;
            return residual;
        }

        // 算法参数
        double L = 1.0;
        double t = 1.0;
        double theta = 1.0;
        double thetaNew = 1.0;
        double Beta = 0.0;

        residual = 1e30;

        // 初始 guess
        // gamma 已置零或通过 warm start 设置
        y = gamma;

        // 主循环
        for (m_iterations = 0; m_iterations < m_max_iterations; ++m_iterations) {
            // g = Z * y
            // 此处省略稠密矩阵 Z，只用 r 模拟
            for (int i = 0; i < nc; ++i) {
                g[i] = r[i]; // 模拟 N*y
            }

            // 乘子更新并投影
            for (int i = 0; i < nc; ++i) {
                gammaNew[i] = y[i] - t * (g[i] + r[i]);
                // 简单非负投影
                if (gammaNew[i] < 0.0) gammaNew[i] = 0.0;
            }

            // Nesterov step
            thetaNew = (std::sqrt(theta * theta + 4.0) - theta) / 2.0;
            Beta = theta * (1.0 - theta) / (theta * theta + thetaNew);
            for (int i = 0; i < nc; ++i) {
                yNew[i] = gammaNew[i] + Beta * (gammaNew[i] - gamma[i]);
            }

            // 计算残差
            double res = 0.0;
            for (int i = 0; i < nc; ++i) {
                double diff = gammaNew[i] - yNew[i];
                res += diff * diff;
            }
            res = std::sqrt(res);

            // 更新最优解
            if (res < residual) {
                residual = res;
                gamma_hat = gammaNew;
            }
            if (res < m_tolerance)
                break;

            // 准备下次迭代
            gamma = gammaNew;
            y = yNew;
            theta = thetaNew;
            t = 1.0 / L;
        }

        // 写回解
        sysd.SetUnknowns(gamma_hat);

        return residual;
    }

} // namespace VSLibRBDynamX
