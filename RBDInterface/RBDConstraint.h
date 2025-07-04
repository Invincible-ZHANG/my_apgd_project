#pragma once

#include <vector>
#include "RBDVariables.h"

namespace VSLibRBDynamX {

    /// 抽象“约束”类（可实现距离约束、接触、摩擦等）
    class RBDConstraint {
    public:
        virtual ~RBDConstraint() {}

        /// 取得约束关联的变量对象（返回引用或指针，支持多体）
        virtual const std::vector<RBDVariables*>& GetVariables() const = 0;

        /// 取得约束维数
        virtual int GetConstraintDim() const = 0;

        /// 计算约束的 Jacobian 矩阵 J（对变量的导数）
        /// J 的尺寸为 [constraintDim x totalDOF]
        virtual void ComputeJacobian(std::vector<std::vector<double>>& J) const = 0;

        /// 计算当前约束右端项（如 phi/h）
        virtual double GetBiasTerm() const = 0;

        /// 投影操作（如摩擦锥的投影，适用于APGD/PGS等）
        /// 输入输出: lambda 长度等于 GetConstraintDim()
        virtual void Project(std::vector<double>& lambda) const = 0;
    };

} // namespace VSLibRBDynamX
