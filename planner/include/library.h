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

struct State {
  float x = 0;
  float y = 0;
  float theta = 0;
};

struct CArray {
  size_t data;
  int size;
};


typedef bool(*ValidStateCallBack)(State&);
typedef bool(*ClosestPointCallBack)(State&, State&);


struct RRTSearchInput {
  State goal;
  State state;
  double radius;
  State planningCenter;
  State planningSize;
  double planTime;
  int pathResolution;
  double dirScalar;
  double lambda;
  double widthScale;
  double numBasisPerMeter;
  ValidStateCallBack validState;
  ClosestPointCallBack closestPoint;
};

struct RRTSearchOutput {
  CArray path;
};


void initROS();

Eigen::VectorXd gaussian_rbf(const Eigen::MatrixXd &x, const Eigen::VectorXd &mu, double sigma);

Eigen::VectorXd gaussian_rb_derivative(const Eigen::MatrixXd &x, const Eigen::VectorXd &mu, double sigma);

std::pair<Eigen::MatrixXd, Eigen::MatrixXd> get_rbf_gradient_basis(int numberBasis, double width, int numPoints, std::pair<double, double> range);

Eigen::MatrixXd get_rbf_basis(Eigen::VectorXd & s, int numberBasis, double width, std::pair<double, double> range);

typedef std::optional<Eigen::MatrixXd> optionalMatrix;
typedef std::optional<Eigen::VectorXd> optionalVector;

Eigen::VectorXd
SolveQP(const Eigen::MatrixXd &H, const Eigen::VectorXd &g, const optionalMatrix &A,
        const optionalVector &lb,
        const optionalVector &ub,
        const optionalVector &lbA, const optionalVector &ubA, qpOASES::int_t nWSR,
        const optionalVector& xOpt);

extern "C" {
//MY_LIB_API bool
//RRTSearch(std::intptr_t handle, std::intptr_t goalPtrIn, std::intptr_t statePtrIn, ValidStateCallBack cb,
//          std::intptr_t *path,
//          int *pathLen, double planTime, double lambda, double widthScale, double numBasisPerMeter);
MY_LIB_API bool RRTSearch(std::intptr_t handle, RRTSearchInput *input, RRTSearchOutput *output);
MY_LIB_API std::intptr_t Init();
MY_LIB_API void Destroy(std::intptr_t handle);
}
