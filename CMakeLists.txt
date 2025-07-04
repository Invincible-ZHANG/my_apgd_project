cmake_minimum_required(VERSION 3.10)
project(MyAPGDTest)

# 使用 C++17
set(CMAKE_CXX_STANDARD 17)

# 把这些目录下的 .h 放入编译器搜索路径
# RBDInterface      -- 放你的 RBDVariables.h / RBDConstraint.h
# Wrapper           -- 放 MyRBDVariables.h / MyRBDConstraint.h
# solver/include    -- 放简化之后的 RBDIterativeSolverVI.h / RBDSolverAPGD.h
# test              -- 放 SimpleSystemDescriptor.h
#
include_directories(
  ${CMAKE_SOURCE_DIR}/RBDInterface
  ${CMAKE_SOURCE_DIR}/Wrapper
  ${CMAKE_SOURCE_DIR}/solver/include
  ${CMAKE_SOURCE_DIR}/test
)

# 只搜集你真正想编译的 solver 源文件，不要把 RBDIterativeSolver.cpp 拉进来
set(SOLVER_SRC
  ${CMAKE_SOURCE_DIR}/solver/src/RBDIterativeSolverVI.cpp
  ${CMAKE_SOURCE_DIR}/solver/src/RBDSolverAPGD.cpp
)

# 最终可执行文件
add_executable(test_apgd
  ${SOLVER_SRC}
  test/main.cpp
)

# （可选）如果以后你还需要加别的源文件，只要 append 到 SOLVER_SRC 或再写 file(GLOB ...) 即可
