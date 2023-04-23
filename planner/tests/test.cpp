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
    Eigen::VectorXd s = Eigen::VectorXd::LinSpaced(numPoints, range.first, range.second);
    Eigen::MatrixXd A = get_rbf_basis(s, numberBasis, width, range);

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
    Eigen::VectorXd s = Eigen::VectorXd::LinSpaced(numPoints, range.first, range.second);
    Eigen::MatrixXd M = get_rbf_basis(s, numberBasis, width, range);

    Eigen::VectorXd target = Eigen::VectorXd::LinSpaced(numPoints, 0, 15);
    target = sin(target.array());

    Eigen::VectorXd lb = -1000000.0 * Eigen::VectorXd::Ones(numberBasis);
    Eigen::VectorXd ub = -lb;
    qpOASES::int_t nWSR = 1E3;

    Eigen::MatrixXd H = M.transpose() * M;
    Eigen::MatrixXd g = -M.transpose() * target;

    Eigen::VectorXd weights = SolveQP(H, g, {}, {lb}, {ub}, {}, {}, nWSR, {});
    Eigen::VectorXd out = M * weights;

    plotty::plot(target);
    plotty::plot(out);

    plotty::show();

}


TEST(QP, fitBasisDerivative) {
    int numberBasis = 10;
    double width = 5;
    int numPoints = 1000;
    std::pair<double, double> range = {-1, 1};
    Eigen::VectorXd s = Eigen::VectorXd::LinSpaced(numPoints, range.first, range.second);

    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(2 * numPoints, 2 * numberBasis);
    Eigen::MatrixXd Mi = get_rbf_basis(s, numberBasis, width, range);
    M.block(0, 0, numPoints, numberBasis) = Mi;
    M.block(numPoints, numberBasis, numPoints, numberBasis) = Mi;

    Eigen::VectorXd target1 = Eigen::VectorXd::LinSpaced(numPoints, 0, 15);
    Eigen::VectorXd target2 = Eigen::VectorXd::LinSpaced(numPoints, 0, 15);
    target1 = sin(target1.array());
    target2 = target2.array();
    Eigen::VectorXd target(target1.size()+target2.size());
    target << target1,target2;

    qpOASES::int_t nWSR = 1E3;

    Eigen::MatrixXd H = M.transpose() * M;
    H = 0.5 * (H + H.transpose()) + .0000000001*Eigen::MatrixXd::Identity(H.cols(), H.cols());
    Eigen::MatrixXd g = -M.transpose() * target;

    auto [Axd, Ayd]  = get_rbf_gradient_basis(numberBasis, width, numPoints, range);
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(1, 2 * numberBasis);
    A.block(0,0,1,numberBasis) = Axd.row(0);
    Eigen::VectorXd lbA(1);
    lbA << 0;
    Eigen::VectorXd ubA(1);
    ubA << 0;

    Eigen::VectorXd weights = SolveQP(H, g, A, {}, {}, lbA, ubA, nWSR, {});
//    Eigen::VectorXd weights = SolveQP(H, g, {}, {}, {}, {}, {}, nWSR);
    Eigen::VectorXd out = M * weights;

    Eigen::MatrixXd val = out.reshaped(numPoints, 2);


    std::cout << "constraint is :" << A*weights;

//    std::cout << "Axd" <<Axd;
//    std::cout << "Ayd" <<Ayd;

    plotty::plot(target1, target2);
    plotty::plot(val.col(0), val.col(1));

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