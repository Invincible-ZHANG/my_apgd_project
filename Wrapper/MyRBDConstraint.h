#pragma once

#include "RBDConstraint.h"
#include "RBDVariables.h"
#include <vector>

namespace VSLibRBDynamX {

    /**
     * MyRBDConstraint
     *   脱离 VEROSIM 的最简实现：
     *   约束形式 C x + b = 0，其中 C = [1, 0, 0, …]，b 可配置
     */
    class MyRBDConstraint : public RBDConstraint {
    public:
        /// @param var  被约束的变量
        /// @param bias 约束偏置 b
        MyRBDConstraint(RBDVariables* var, double bias = 0.0)
            : m_var(var), m_bias(bias) {}

        ~MyRBDConstraint() override = default;

        /// 返回本约束关联的所有变量（这里只有一个）
        const std::vector<RBDVariables*>& GetVariables() const override {
            static std::vector<RBDVariables*> tmp;
            tmp = { m_var };
            return tmp;
        }

        /// 约束维度为 1
        int GetConstraintDim() const override { return 1; }

        /// 构造 1×DOF 的 Jacobian，只有第一列为 1
        void ComputeJacobian(std::vector<std::vector<double>>& J) const override {
            int dof = m_var->GetDOF();
            J.assign(1, std::vector<double>(dof, 0.0));
            J[0][0] = 1.0;
        }

        /// 返回偏置 b
        double GetBiasTerm() const override {
            return m_bias;
        }

        /// 对 λ 做非负投影
        void Project(std::vector<double>& lambda) const override {
            if (!lambda.empty() && lambda[0] < 0.0) lambda[0] = 0.0;
        }

    private:
        RBDVariables* m_var;  ///< 被约束的变量指针
        double        m_bias; ///< 约束偏置项 b
    };

} // namespace VSLibRBDynamX
