#include "library.h"
#include <Eigen/Core>
#include <gtest/gtest.h>
#include <plotty/matplotlibcpp.hpp>


TEST(OMPL, search) {
    initROS();
    ASSERT_TRUE(true);
    std::cout << "run good" << '\n';

}

TEST(QP, rbf) {
    int numberBasis = 10;
    double width = .5;
    int numPoints = 100;
    std::pair<double, double> range = {-1, 1};

    Eigen::MatrixXd A = get_rbf_basis(numberBasis, width, numPoints, range);
    for (auto i = 0; i < A.cols(); i++) {
        plotty::plot(A.col(i));
    }
    plotty::show();

}


TEST(QP, fitBasis) {
    int numberBasis = 10;
    double width = .5;
    int numPoints = 100;
    std::pair<double, double> range = {-1, 1};

    Eigen::MatrixXd M = get_rbf_basis(numberBasis, width, numPoints, range);

    Eigen::VectorXd target = Eigen::VectorXd::LinSpaced(numPoints, 0, 15);
    target = sin(target.array());

    Eigen::VectorXd lb = -1000000.0 * Eigen::VectorXd::Ones(numberBasis);
    Eigen::VectorXd ub = -lb;
    qpOASES::int_t nWSR = 1E3;

    Eigen::MatrixXd H = M.transpose() * M;
    Eigen::MatrixXd g = -M.transpose() * target;

    Eigen::VectorXd weights = SolveQP(H, g, {}, {lb}, {ub}, {}, {}, nWSR);
    Eigen::VectorXd out = M * weights;

    plotty::plot(target);
    plotty::plot(out);

    plotty::show();

}


TEST(PLOTTY, simple) {
    std::vector<double> v({1, 2, 3, 4});
    plotty::plot(v);
    plotty::show();
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}