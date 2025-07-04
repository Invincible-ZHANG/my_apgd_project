#pragma once
#include "../RBDInterface/RBDSystemDescriptor.h"
#include "../Wrapper/MyRBDConstraint.h"
#include "../Wrapper/MyRBDVariables.h"
#include <vector>
#include <cassert>

namespace VSLibRBDynamX {

    /// 一个非常简单的系统描述器：1 个变量、1 个约束
    class SimpleSystemDescriptor : public RBDSystemDescriptor {
    public:
        void AddVariables(RBDVariables* v) override {
            vars.push_back(v);
        }
        void AddConstraint(RBDConstraint* c) override {
            cons.push_back(c);
        }
        const std::vector<RBDVariables*>& GetVariables() const override {
            return vars;
        }
        const std::vector<RBDConstraint*>& GetConstraints() const override {
            return cons;
        }

        // 构建 Z (1×1) 和 d (1)
        void BuildSystemMatrix(std::vector<std::vector<double>>& Z,
            std::vector<double>& d) const override {
            int n = 1;
            Z.assign(n, std::vector<double>(n, 0.0));
            d.assign(n, 0.0);

            // 简化：Z = [1.0]，d = [bias]
            Z[0][0] = 1.0;
            // 把所有约束的 bias 累加到 d
            for (auto* c : cons)
                d[0] += c->GetBiasTerm();
        }

        // y = Z * x
        void SystemProduct(const std::vector<double>& x,
            std::vector<double>& y) const override {
            assert(x.size() == 1);
            y.resize(1);
            y[0] = x[0] * 1.0;
        }

        // 只构建 bias 部分
        void BuildDiVector(std::vector<double>& di) const override {
            di.assign(1, 0.0);
            for (auto* c : cons)
                di[0] += c->GetBiasTerm();
        }

        // 将解写回变量
        void SetUnknowns(const std::vector<double>& sol) override {
            assert(sol.size() == 1);
            vars[0]->SetState(sol);
        }

    private:
        std::vector<RBDVariables*> vars;
        std::vector<RBDConstraint*> cons;
    };

} // namespace
