#pragma once

#include "RBDVariables.h"
#include <vector>

namespace VSLibRBDynamX {

    /**
     * MyRBDVariables
     *   脱离 VEROSIM 的最简实现：封装一个只有 1 自由度的 SimpleRigidBody
     */
    class MyRBDVariables : public RBDVariables {
    public:
        /// @param mass 质量 m
        MyRBDVariables(double mass = 2.0) : m_mass(mass), m_state(0.0) {}

        /// 这里只有 1 个自由度
        int GetDOF() const override { return 1; }

        /// 返回当前状态 x
        void GetState(std::vector<double>& x) const override {
            x.resize(1);
            x[0] = m_state;
        }

        /// 设置状态 x
        void SetState(const std::vector<double>& x) override {
            if (!x.empty()) m_state = x[0];
        }

        /// M^{-1} * f，只做标量运算
        void ComputeMassInverseTimesVector(const std::vector<double>& f,
            std::vector<double>& result) const override {
            result.resize(1);
            // 防止除零
            result[0] = (m_mass > 0) ? (f[0] / m_mass) : 0.0;
        }

    private:
        double m_mass;   ///< 质量
        double m_state;  ///< 单自由度的状态值（例如速度）
    };

} // namespace VSLibRBDynamX
