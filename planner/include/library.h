#pragma once

#include "Eigen/Core"
#include <qpOASES/QProblem.hpp>
#include <optional>

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

Eigen::VectorXd gaussian_rbf(const Eigen::MatrixXd &x, const Eigen::VectorXd &mu, double sigma);

Eigen::MatrixXd get_rbf_basis(int numberBasis, double width, int numPoints, std::pair<double, double> range);

typedef std::optional<Eigen::MatrixXd> optionalMatrix;
typedef std::optional<Eigen::VectorXd> optionalVector;

Eigen::VectorXd
SolveQP(const Eigen::MatrixXd &H, const Eigen::VectorXd &g, const optionalMatrix &A, const optionalVector &lb,
      const optionalVector &ub,
      const optionalVector &lbA, const optionalVector &ubA, qpOASES::int_t nWSR);

extern "C" {
MY_LIB_API bool
RRTSearch(std::intptr_t handle, std::intptr_t goalPtrIn, std::intptr_t statePtrIn, FuncCallBack cb, std::intptr_t *path,
          int *pathLen, double turnRadius, double planTime);
MY_LIB_API std::intptr_t Init();
MY_LIB_API void Destroy(std::intptr_t handle);
}
