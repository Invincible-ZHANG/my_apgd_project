#pragma once

#include <vector>
#include "RBDVariables.h"
#include "RBDConstraint.h"

namespace VSLibRBDynamX {

    /**
     * 管理整个系统的变量、约束集合，
     * 并提供组装全局系统矩阵、乘法、和写回解的接口
     */
    class RBDSystemDescriptor {
    public:
        virtual ~RBDSystemDescriptor() {}

        /// 增加一个变量
        virtual void AddVariables(RBDVariables* vars) = 0;

        /// 增加一个约束
        virtual void AddConstraint(RBDConstraint* constraint) = 0;

        /// 获取所有变量对象
        virtual const std::vector<RBDVariables*>& GetVariables() const = 0;

        /// 获取所有约束对象
        virtual const std::vector<RBDConstraint*>& GetConstraints() const = 0;

        /**
         * 构建全局系统矩阵 Z 和右端向量 d，使得 Z * x = d
         * @param Z 输出：大小为 n×n 的矩阵（这里用稠密存储，实际可替换为稀疏格式）
         * @param d 输出：长度为 n 的右端向量
         */
        virtual void BuildSystemMatrix(std::vector<std::vector<double>>& Z,
            std::vector<double>& d) const = 0;

        /**
         * 计算矩阵-向量乘积 y = Z * x
         * @param x 输入：长度为 n 的向量
         * @param y 输出：长度为 n 的结果向量
         */
        virtual void SystemProduct(const std::vector<double>& x,
            std::vector<double>& y) const = 0;

        /**
         * 构建仅包含约束偏置项 b 的向量 di（可用于 APGD 中将 d 拆成 f+b）
         * @param di 输出：长度为 n 的“偏置”向量
         */
        virtual void BuildDiVector(std::vector<double>& di) const = 0;

        /**
         * 将求解得到的解向量 x = [q; -λ] 写回到各个变量和约束中
         * @param x 输入：长度为 n 的解向量
         */
        virtual void SetUnknowns(const std::vector<double>& x) = 0;
    };

} // namespace VSLibRBDynamX
