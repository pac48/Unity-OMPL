#pragma once

#include "Eigen/Core"
#include <qpOASES/QProblem.hpp>

#if defined(_MSC_VER)
#define MY_LIB_API __declspec(dllexport) // Microsoft
#elif defined(__GNUC__)
#define MY_LIB_API __attribute__((visibility("default"))) // GCC
#else
#define MY_LIB_API // Most compilers export all the symbols by default. We hope for the best here.
#pragma warning Unknown dynamic link import/export semantics.
#endif

typedef bool(*FuncCallBack)();

void initROS();
Eigen::VectorXd gaussian_rbf(const Eigen::MatrixXd & x, const Eigen::VectorXd & mu, double sigma);

extern "C" {
MY_LIB_API bool
RRTSearch(std::intptr_t handle, std::intptr_t goalPtrIn, std::intptr_t statePtrIn, FuncCallBack cb, std::intptr_t *path,
          int *pathLen, double turnRadius, double planTime);
MY_LIB_API std::intptr_t Init();
MY_LIB_API void Destroy(std::intptr_t handle);
}
