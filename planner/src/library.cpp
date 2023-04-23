#include <ompl/base/Planner.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/cforest/CForest.h>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "library.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace std::chrono_literals;


#include <iostream>
#include <vector>


class MinimalPublisher : public rclcpp::Node {
public:
    MinimalPublisher()
            : Node("minimal_publisher"), count_(0) {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    }

    void publish(const std::string &data) {
        auto message = std_msgs::msg::String();
        message.data = data;
        publisher_->publish(message);
    }

private:
    void timer_callback() {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

bool rosInitialized = false;

void initROS() {
    if (!rosInitialized) {
        auto name = "ros_node";
        rclcpp::init(1, &name);

        auto myThread = new std::thread([]() {
            rclcpp::spin(std::make_shared<MinimalPublisher>());
            rclcpp::shutdown();

        });

        rosInitialized = true;
    }
}

Eigen::VectorXd gaussian_rbf(const Eigen::MatrixXd &x, const Eigen::VectorXd &mu, double sigma) {
    auto delta = (x.rowwise() - mu.transpose());
    auto delta2 = delta.array() * delta.array();
    auto dist = delta2.rowwise().sum();

    return exp(-dist / (2 * sigma));

}

Eigen::VectorXd gaussian_rb_derivative(const Eigen::MatrixXd &x, const Eigen::VectorXd &mu, double sigma) {
    Eigen::MatrixXd delta = (x.rowwise() - mu.transpose());
    Eigen::MatrixXd delta2 = delta.array() * delta.array();
    Eigen::VectorXd dist = delta2.rowwise().sum();

    assert(delta.cols() == 1);
    Eigen::VectorXd xd = (-1.0 / sigma) * delta.col(0).array() * exp(-dist.array() / (2 * sigma));
    //Eigen::VectorXd yd  = (-1.0/sigma)*delta.col(1).array()*exp(-dist.array() / (2 * sigma));
    return xd;
}


Eigen::MatrixXd get_rbf_basis(Eigen::VectorXd &s, int numberBasis, double width, std::pair<double, double> range) {
    int numPoints = s.size();
    Eigen::MatrixXd A = Eigen::MatrixXd::Ones(numPoints, numberBasis);
    Eigen::VectorXd centers = Eigen::VectorXd::LinSpaced(numberBasis - 1, range.first, range.second);
    Eigen::VectorXd mu = Eigen::VectorXd::Zero(1);
    int ind = 0;
    for (auto c: centers) {
        mu[0] = c;
        A.col(ind) = gaussian_rbf(s, mu, width);
        ind++;
    }
    A = A.unaryExpr([](double v) { return std::isfinite(v) ? v : 0.0; });

    return A;
}

std::pair<Eigen::MatrixXd, Eigen::MatrixXd>
get_rbf_gradient_basis(int numberBasis, double width, int numPoints, std::pair<double, double> range) {

    Eigen::MatrixXd Adx = Eigen::MatrixXd::Zero(numPoints, numberBasis);
    Eigen::MatrixXd Ady = Eigen::MatrixXd::Zero(numPoints, numberBasis);

    Eigen::VectorXd centers = Eigen::VectorXd::LinSpaced(numberBasis - 1, range.first, range.second);
    Eigen::VectorXd mu = Eigen::VectorXd::Zero(1);
    Eigen::MatrixXd x = Eigen::MatrixXd::Zero(numPoints, 1);
    int ind = 0;
    for (auto c: centers) {
        mu[0] = c;
        x.col(0) = Eigen::VectorXd::LinSpaced(numPoints, range.first, range.second);

        auto out = gaussian_rb_derivative(x, mu, width);
        Adx.col(ind) = out;
        Ady.col(ind) = out;

        ind++;
    }
    Adx = Adx.unaryExpr([](double v) { return std::isfinite(v) ? v : 0.0; });
    Ady = Ady.unaryExpr([](double v) { return std::isfinite(v) ? v : 0.0; });

    return {Adx, Ady};
}

Eigen::VectorXd
SolveQP(const Eigen::MatrixXd &H, const Eigen::VectorXd &g, const optionalMatrix &A,
        const optionalVector &lb,
        const optionalVector &ub,
        const optionalVector &lbA, const optionalVector &ubA, qpOASES::int_t nWSR,
        const optionalVector& xOpt) {
    using namespace qpOASES;

    /* Setup data of first QP. */
    auto H_arr = H.data();
    auto g_arr = g.data();
    const real_t *A_arr = nullptr;
    const real_t *lb_arr = nullptr;
    const real_t *ub_arr = nullptr;
    const real_t *lbA_arr = nullptr;
    const real_t *ubA_arr = nullptr;
    const real_t *xOpt_arr = nullptr;
    Eigen::MatrixXd AT;
    int nC = 0;
    if (A.has_value()) {
        nC = A->rows();
        AT = A.value().transpose();
        A_arr = AT.data();
    }
    if (lb.has_value()) {
        lb_arr = lb.value().data();
    }
    if (ub.has_value()) {
        ub_arr = ub.value().data();
    }
    if (lbA.has_value()) {
        lbA_arr = lbA.value().data();
    }
    if (ubA.has_value()) {
        ubA_arr = ubA.value().data();
    }
    if (xOpt.has_value()) {
        xOpt_arr = xOpt.value().data();
    }


    qpOASES::QProblem example(H.cols(), nC, qpOASES::HessianType::HST_POSDEF);
    qpOASES::Options options;
    options.setToReliable();
    options.enableEqualities = qpOASES::BooleanType::BT_TRUE;
    example.setOptions(options);

    /* Solve first QP. */
//    if (!warmStart){
    example.init(H_arr, g_arr, A_arr, lb_arr, ub_arr, lbA_arr, ubA_arr, nWSR, nullptr, xOpt_arr, nullptr);
//    } else{
//        example.hotstart( g_new,lb_new,ub_new,lbA_new,ubA_new, nWSR );
//    }

    /* Get and print solution of first QP. */
    real_t weights_arr[H.cols()];
    example.getPrimalSolution(weights_arr);

    Eigen::VectorXd weights = Eigen::Map<Eigen::VectorXd>(weights_arr, H.cols());
    return weights;
}

class UnityOMPLInterface {
public:
    int
    smoothPath(std::vector<State> &pathVec, RRTSearchInput *input) {
        std::stringstream sstream;
        MinimalPublisher pubNode;

        auto pathVecCopy = pathVec;
        Eigen::VectorXd tmp = Eigen::Map<Eigen::VectorXf>((float *) pathVecCopy.data(),
                                                          3 * pathVecCopy.size()).cast<double>();
        Eigen::MatrixXd path = tmp.reshaped(3, pathVec.size()).transpose();
        Eigen::MatrixXd pathDiffs = path.middleRows(1, path.rows() - 1) - path.middleRows(0, path.rows() - 1);
        Eigen::VectorXd dist = (pathDiffs.cwiseProduct(pathDiffs)).rowwise().sum().array().sqrt();
        Eigen::VectorXd pathlength = Eigen::VectorXd::Zero(pathVec.size());
        std::partial_sum(dist.begin(), dist.end(), pathlength.begin() + 1, std::plus<double>());

        double width = input->widthScale;
        int numPoints = pathlength.size();
        std::pair<double, double> range = {0.0, pathlength.tail(1)(0)};
        int numberBasis = input->numBasisPerMeter * range.second + 6; // add some extra to make problem feasible
        Eigen::VectorXd s = Eigen::VectorXd::LinSpaced(numPoints, range.first, range.second);

        Eigen::MatrixXd Mi = get_rbf_basis(s, numberBasis, width, range);
        Eigen::MatrixXd M = Eigen::MatrixXd::Zero(2 * numPoints + 2 * numberBasis, 2 * numberBasis);
        M.block(0, 0, numPoints, numberBasis) = Mi;
        M.block(numPoints, 0, numberBasis, numberBasis) =
                input->lambda * Eigen::MatrixXd::Identity(numberBasis, numberBasis);
        M.block(numPoints + numberBasis, numberBasis, numPoints, numberBasis) = Mi;
        M.block(2 * numPoints + numberBasis, numberBasis, numberBasis, numberBasis) =
                input->lambda * Eigen::MatrixXd::Identity(numberBasis, numberBasis);

        Eigen::VectorXd target(2 * numPoints + 2 * numberBasis);
        target << path.col(0), Eigen::VectorXd::Zero(numberBasis), path.col(1), Eigen::VectorXd::Zero(numberBasis);

        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(4, 2 * numberBasis);
        Eigen::VectorXd lbA(4);
        Eigen::VectorXd ubA(4);
        // add constraints to enforce start and goal positions
        A.block(0, 0, 1, numberBasis) = Mi.row(0);
        A.block(1, 0, 1, numberBasis) = Mi.row(numPoints - 1);
        A.block(2, numberBasis, 1, numberBasis) = Mi.row(0);
        A.block(3, numberBasis, 1, numberBasis) = Mi.row(numPoints - 1);

        // add constraints to enforce start and goal rotations
        double xDirStart = range.second * cos(input->state.theta);
        double yDirStart = range.second * sin(input->state.theta);
        double xDirGoal = range.second * cos(input->goal.theta);
        double yDirGoal = range.second * sin(input->goal.theta);
        auto [Mxd, Myd] = get_rbf_gradient_basis(numberBasis, width, numPoints, range);

//        A.block(4, 0, 1, numberBasis) = -yDirStart * Mxd.row(0);
//        A.block(4, numberBasis, 1, numberBasis) = xDirStart * Myd.row(0);
//        A.block(5, 0, 1, numberBasis) = -yDirGoal * Mxd.row(numPoints - 1);
//        A.block(5, numberBasis, 1, numberBasis) = xDirGoal * Myd.row(numPoints - 1);
//
//        A.block(6, 0, 1, numberBasis) = xDirStart * Mxd.row(0);
//        A.block(6, numberBasis, 1, numberBasis) = yDirStart * Myd.row(0);
//        A.block(7, 0, 1, numberBasis) = xDirGoal * Mxd.row(numPoints - 1);
//        A.block(7, numberBasis, 1, numberBasis) = yDirGoal * Myd.row(numPoints - 1);
//        double eps = input->dirScalar;
//
        lbA << pathVec[0].x, pathVec.back().x, pathVec[0].y, pathVec.back().y;//,
//                0.0, 0.0, eps, eps;
        ubA << pathVec[0].x, pathVec.back().x, pathVec[0].y, pathVec.back().y;//,
//                0.0, 0.0, range.second * 1000000, range.second * 1000000;

        Eigen::MatrixXd H = M.transpose() * M;
        H = 0.5 * (H + H.transpose());
        Eigen::MatrixXd g = -M.transpose() * target;

        /* Setting up QProblem object. */
        qpOASES::int_t nWSR = 1E3;
        std::optional<Eigen::VectorXd> xOpt;

        Eigen::MatrixXd pathSmooth;
        for (auto it = 0; it < 100; it++) {
            Eigen::VectorXd weights = SolveQP(H, g, A, {}, {}, lbA, ubA, nWSR, xOpt);
            Eigen::VectorXd out = M * weights;
            xOpt = out;
            pathSmooth = out.reshaped(numPoints + numberBasis, 2);

            // TODO!!!!!!!!!
            State curPoint;
            State tmpPoint;
            std::tuple<State, State, double> closestPair;
            bool violation = false;
            double minD = 1E100;
            for (int i = 0; i < pathVec.size(); i++) {
                curPoint.x = pathSmooth(i, 0);
                curPoint.y = pathSmooth(i, 1);
                if (input->closestPoint(curPoint, tmpPoint)) {
                    double diffX = (tmpPoint.x - curPoint.x);
                    double diffY = (tmpPoint.y - curPoint.y);
                    double d = diffX * diffX + diffY * diffY;
                    if (d < minD) {
                        minD = d;
                        std::get<0>(closestPair) = pathVec[i];
                        std::get<1>(closestPair) = tmpPoint;
                        std::get<2>(closestPair) = pathlength[i];
                        violation = true;
                    }
                }
            }

            if (violation) {
                Eigen::MatrixXd ACopy = A;
                A = Eigen::MatrixXd(A.rows() + 1, A.cols());
                A.block(0, 0, ACopy.rows(), ACopy.cols()) = ACopy;

                Eigen::MatrixXd lbACopy = lbA;
                Eigen::MatrixXd ubACopy = ubA;
                lbA = Eigen::VectorXd(lbA.size() + 1);
                ubA = Eigen::VectorXd(ubA.size() + 1);
                lbA.block(0, 0, lbACopy.rows(), 1) = lbACopy;
                ubA.block(0, 0, ubACopy.rows(), 1) = ubACopy;

                auto pathPoint = std::get<0>(closestPair);
                auto closestPoint = std::get<1>(closestPair);
                double dirX = (pathPoint.x - closestPoint.x);
                double dirY = (pathPoint.y - closestPoint.y);
                double norm = sqrt(dirX * dirX + dirY * dirY);
                dirX = dirX / norm;
                dirY = dirY / norm;

                double si = std::get<2>(closestPair);
                Eigen::VectorXd siVec(1);
                siVec << si;
                auto Ai = get_rbf_basis(siVec, numberBasis, width, range);
                auto near = input->radius + dirX * closestPoint.x + dirY * closestPoint.y;;

                A.block(ACopy.rows(), 0, 1, numberBasis) = Ai * dirX;
                A.block(ACopy.rows(), numberBasis, 1, numberBasis) = Ai * dirY;
                lbA[lbACopy.rows()] = near;
                ubA[ubACopy.rows()] = range.second * 1000000;

            } else {
                break;
            }
        }
        for (int i = 0; i < pathVec.size(); i++) {
            pathVec[i].x = pathSmooth(i, 0);
            pathVec[i].y = pathSmooth(i, 1);
        }

//        pubNode.publish(sstream.str());

        return 0;
    }

    bool
    RRTSearch(RRTSearchInput *input, RRTSearchOutput *output) {
        if (!input->validState(input->goal)) {
            return false;
        }

        initROS();

        ob::RealVectorBounds bounds(2);
        bounds.low[0] = input->planningCenter.x - .5 * input->planningSize.x;
        bounds.low[1] = input->planningCenter.y - .5 * input->planningSize.y;
//    bounds.low[2] = -0.001;
        bounds.high[0] = input->planningCenter.x + .5 * input->planningSize.x;
        bounds.high[1] = input->planningCenter.y + .5 * input->planningSize.y;
//    bounds.high[2] = 0.001;
        auto space(std::make_shared<ob::RealVectorStateSpace>(2));


        space->setBounds(bounds);

        og::SimpleSetup ss(space);

        auto si = ss.getSpaceInformation();
        si->setStateValidityChecker([=](const ob::State *state) {
            const auto *s = state->as<ob::RealVectorStateSpace::StateType>();
            State test;
            test.x = s->values[0];
            test.y = s->values[1];

            return input->validState(test);
        });

        ob::ScopedState<> start(space);
        start[0] = input->state.x;
        start[1] = input->state.y;

        ob::ScopedState<> goal(space);
        goal[0] = input->goal.x;
        goal[1] = input->goal.y;

        auto planner(std::make_shared<ompl::geometric::RRTConnect>(si));
        ss.setPlanner(planner);

        ss.setStartAndGoalStates(start, goal);
        ss.getSpaceInformation()->setStateValidityCheckingResolution(0.01);
        ss.setup();

        bool solved = ss.solve(input->planTime);

        std::vector<ompl::base::State *> solution;
        if (solved) {
            std::cout << "Found solution:" << std::endl;
            fprintf(stderr, "Found solution:");

            ss.simplifySolution();
            ss.getSolutionPath().print(std::cout);
            std::stringstream sstream;
            ss.getSolutionPath().print(sstream);

            og::PathGeometric solPath = ss.getSolutionPath();
            solPath.interpolate(input->pathResolution);
            solution = solPath.getStates();

            if (out) {
                delete out;
            }
            out = new State[solution.size()];
            for (auto i = 0; i < solution.size(); i++) {
                auto s = solution[i]->as<ob::RealVectorStateSpace::StateType>();
                out[i].x = s->values[0];
                out[i].y = s->values[1];
                out[i].theta = 0.0; // TODO this is wrong
            }
            std::vector<State> pathVec(out, out + solution.size());
            smoothPath(pathVec, input);

            memcpy(out, pathVec.data(), pathVec.size() * sizeof(State));
            output->path.data = (size_t) out;
            output->path.size = pathVec.size();
        }

        return solved;
    }

private:
    State *out = nullptr;

};

std::intptr_t Init() {
    return (std::intptr_t) new UnityOMPLInterface();
}

void Destroy(std::intptr_t handle) {
    auto ptr = (UnityOMPLInterface *) handle;
    delete ptr;
}

bool RRTSearch(std::intptr_t handle, RRTSearchInput *input, RRTSearchOutput *output) {
    auto ptr = (UnityOMPLInterface *) handle;
    return ptr->RRTSearch(input, output);
}



