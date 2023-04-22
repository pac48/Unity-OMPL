#include "library.h"
#include <Eigen/Core>
#include <gtest/gtest.h>
#include <plotty/matplotlibcpp.hpp>
#include <Eigen/Dense>

TEST(OMPL, search) {
    initROS();
    ASSERT_TRUE(true);
    std::cout << "run good" << '\n';

}

TEST(QP, rbf) {
  auto mu = Eigen::VectorXd::Ones(5);
  Eigen::MatrixXd x = .5*Eigen::MatrixXd::Ones(100, 5);
  x.array() -= 1.0;

  Eigen::VectorXd v = Eigen::VectorXd::LinSpaced(100, 0, 3);
  auto x2 = x.colwise() + v;

  auto out = gaussian_rbf(x2, mu, .5);
  std::cout << "rbf: " << out<< '\n';

  plotty::plot(out);
  plotty::show();

}

TEST(PLOTTY, simple) {
// Simple:
  std::vector<double> v({1, 2, 3, 4});
  plotty::plot(v);
  plotty::show();
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}