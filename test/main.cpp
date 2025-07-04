#include <iostream>
#include <vector>

#include "../Wrapper/MyRBDVariables.h"
#include "../Wrapper/MyRBDConstraint.h"
#include "SimpleSystemDescriptor.h"
#include "../solver/include/RBDSolverAPGD.h"

using namespace VSLibRBDynamX;

int main() {
    // 1) 定义一个单自由度变量，质量 m = 2.0
    MyRBDVariables var(2.0);

    // 2) 定义一个简单约束 C x + b = 0，这里 C = [1]，b = bias
    //    约束方程: x + bias = 0 => x = -bias
    double bias = -4.0;
    MyRBDConstraint cons(&var, bias);

    // 3) 构建系统描述符，并加入变量和约束
    SimpleSystemDescriptor sys;
    sys.AddVariables(&var);
    sys.AddConstraint(&cons);

    // 4) 构造 APGD 求解器并设置参数
    RBDSolverAPGD solver;
    solver.SetMaxIterations(100);      // 最大迭代次数
    solver.SetTolerance(1e-6);         // 收敛阈值（投影梯度范数）
    solver.SetOmega(1.0);              // Over-relaxation 因子 ω
    solver.SetSharpnessLambda(1.0);    // Sharpness 因子 λ
    solver.SetRecordViolation(false);  // 不记录每轮残差历史

    // 5) 运行求解，返回最终残差
    double residual = solver.Solve(sys);

    // 6) 从变量中读取解并打印
    std::vector<double> sol;
    var.GetState(sol);
    double x = sol.empty() ? 0.0 : sol[0];

    std::cout << "APGD converged with residual = " << residual << "\n";
    std::cout << "Solution x = " << x
        << " (expected = " << -bias << ")\n";

    return 0;
}
