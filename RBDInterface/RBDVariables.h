#pragma once

namespace VSLibRBDynamX {

    /// 抽象“变量”类（类似于刚体、柔体的运动学/动力学自由度）
    class RBDVariables {
    public:
        virtual ~RBDVariables() {}

        /// 得到自由度个数
        virtual int GetDOF() const = 0;

        /// 取得当前状态向量（速度、加速度等，返回值类型可用 Eigen::VectorXd 或 std::vector<double>）
        virtual void GetState(std::vector<double>& x) const = 0;

        /// 设置状态向量
        virtual void SetState(const std::vector<double>& x) = 0;

        /// 计算 M^{-1} * 力（实现APGD等需要）
        virtual void ComputeMassInverseTimesVector(const std::vector<double>& f, std::vector<double>& result) const = 0;
    };

} // namespace VSLibRBDynamX
