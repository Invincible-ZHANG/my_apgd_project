#pragma once

#include "RBDVariables.h"
#include "RBDConstraint.h"

namespace VSLibRBDynamX {

    /// 管理整个系统的变量、约束集合，提供向量组装和转换功能
    class RBDSystemDescriptor {
    public:
        virtual ~RBDSystemDescriptor() {}

        /// 增加变量
        virtual void AddVariables(RBDVariables* vars) = 0;

        /// 增加约束
        virtual void AddConstraint(RBDConstraint* constraint) = 0;

        /// 取得所有变量对象
        virtual const std::vector<RBDVariables*>& GetVariables() const = 0;

        /// 取得所有约束对象
        virtual const std::vector<RBDConstraint*>& GetConstraints() const = 0;

        // ... 还可以定义“组装全局向量”“变量写回/同步”等接口
    };

} // namespace VSLibRBDynamX
