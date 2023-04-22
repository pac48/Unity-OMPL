#include <ompl/base/Planner.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/util/Time.h>


#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "library.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace std::chrono_literals;


#include <iostream>
#include <vector>
#include <plotty/matplotlibcpp.hpp>


struct State {
    float x;
    float y;
    float z;
};


class MinimalPublisher : public rclcpp::Node {
public:
    MinimalPublisher()
            : Node("minimal_publisher"), count_(0) {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
//        timer_ = this->create_wall_timer(
//                500ms, std::bind(&MinimalPublisher::timer_callback, this));
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

    return exp(-dist / (2 * sigma * sigma));

}

Eigen::MatrixXd get_rbf_basis(int numberBasis, double width, int numPoints, std::pair<double, double> range) {

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(numPoints, numberBasis);

    Eigen::VectorXd centers = Eigen::VectorXd::LinSpaced(numberBasis, range.first, range.second);
    Eigen::VectorXd mu = Eigen::VectorXd::Zero(1);
    Eigen::MatrixXd x = Eigen::MatrixXd::Zero(numPoints, 1);
    int ind = 0;
    for (auto c: centers) {
        mu[0] = c;
        x.col(0) = Eigen::VectorXd::LinSpaced(numPoints, range.first, range.second);
        A.col(ind) = gaussian_rbf(x, mu, width);
        ind++;
    }


    return A;
}

Eigen::VectorXd
SolveQP(const Eigen::MatrixXd &H, const Eigen::VectorXd &g, const optionalMatrix &A, const optionalVector &lb,
        const optionalVector &ub,
        const optionalVector &lbA, const optionalVector &ubA, qpOASES::int_t nWSR) {
    using namespace qpOASES;

    /* Setup data of first QP. */
    auto H_arr = H.data();
    auto g_arr = g.data();
    const real_t *A_arr = nullptr;
    const real_t *lb_arr = nullptr;
    const real_t *ub_arr = nullptr;
    const real_t *lbA_arr = nullptr;
    const real_t *ubA_arr = nullptr;
    if (A.has_value()) {
        A_arr = A.value().data();
    }
    if (lb.has_value()) {
        lb_arr = lb.value().data();
    }
    if (ub.has_value()) {
        ub_arr = ub.value().data();
    }
    if (lbA.has_value()) {
        lbA_arr = lb.value().data();
    }
    if (ubA.has_value()) {
        ubA_arr = ub.value().data();
    }

    /* Setting up QProblem object. */
    QProblem example(H.cols(), 0);

    Options options;
    example.setOptions(options);

    /* Solve first QP. */
    example.init(H_arr, g_arr, A_arr, lb_arr, ub_arr, lbA_arr, ubA_arr, nWSR);

    /* Get and print solution of first QP. */
    real_t xOpt_arr[H.cols()];
    example.getPrimalSolution(xOpt_arr);

    Eigen::VectorXd weights = Eigen::Map<Eigen::VectorXd>(xOpt_arr, H.cols());
    return weights;
}

class UnityOMPLInterface {
public:
    int testStuff(MinimalPublisher &pubNode, const std::vector<State> &pathVec) {
        std::stringstream sstream;

        Eigen::VectorXd tmp = Eigen::Map<Eigen::VectorXf>((float *) pathVec.data(), 3*pathVec.size()).cast<double>();
        Eigen::MatrixXd path = tmp.reshaped( 3, pathVec.size()).transpose();
        Eigen::MatrixXd pathDiffs = path.middleRows(1, path.rows()-1) - path.middleRows(0, path.rows()-1);
        Eigen::VectorXd dist = (pathDiffs.cwiseProduct(pathDiffs)).rowwise().sum().array().sqrt();
        Eigen::VectorXd pathlength = Eigen::VectorXd::Zero(pathVec.size());
        std::partial_sum(dist.begin(), dist.end(), pathlength.begin()+1, std::plus<double>());

        int numberBasis = pathVec.size();
        double width = .5/pathlength.tail(1)(0);
        int numPoints = pathlength.size();
        std::pair<double, double> range = {0.0, pathlength.tail(1)(0)};

        Eigen::MatrixXd M = get_rbf_basis(numberBasis, width, numPoints, range);

        Eigen::VectorXd target = Eigen::VectorXd::LinSpaced(numPoints, 0, 15);
        target = sin(target.array());


        Eigen::MatrixXd H = M.transpose() * M;
        Eigen::MatrixXd g = -M.transpose() * target;

        qpOASES::int_t nWSR = 1E3;
        Eigen::VectorXd weights = SolveQP(H, g, {}, {}, {}, {}, {}, nWSR);
        Eigen::VectorXd out = M * weights;

//        plotty::plot(target);
//        plotty::plot(out);

        sstream << "xOpt = [" << pathlength << "];" << std::endl;

        pubNode.publish(sstream.str());

        return 0;
    }

    bool
    RRTSearch(State *goalPtr, State *statePtr, FuncCallBack isStateValid, State **path, int *pathLen, double turnRadius,
              double planTime) {
        initROS();
        MinimalPublisher pubNode;

//        ob::RealVectorBounds bounds(2);
//        bounds.low[0] = -5;
//        bounds.low[1] = -5;
//        bounds.high[0] = 5;
//        bounds.high[1] = 5;
//        auto space(std::make_shared<ob::DubinsStateSpace>(turnRadius, false));

        ob::RealVectorBounds bounds(3);
        bounds.low[0] = -5;
        bounds.low[1] = -5;
        bounds.low[2] = -0.001;
        bounds.high[0] = 5;
        bounds.high[1] = 5;
        bounds.high[2] = 0.001;
        auto space(std::make_shared<ob::RealVectorStateSpace>(3));


        space->setBounds(bounds);

        og::SimpleSetup ss(space);

        auto si = ss.getSpaceInformation();
        si->setStateValidityChecker([=](const ob::State *state) {
//            const auto *s = state->as<ob::SE2StateSpace::StateType>();
//            statePtr->x = s->getX();
//            statePtr->y = s->getY();
//            statePtr->z = s->getYaw();
            const auto *s = state->as<ob::RealVectorStateSpace::StateType>();
            statePtr->x = s->values[0];
            statePtr->y = s->values[1];
            statePtr->z = s->values[2];

            return isStateValid();
        });
//        si->setStateValidityCheckingResolution(0.005);

        ob::ScopedState<> start(space);
        start[0] = statePtr->x;
        start[1] = statePtr->y;
        start[2] = statePtr->z;

        ob::ScopedState<> goal(space);
        goal[0] = goalPtr->x;
        goal[1] = goalPtr->y;
        goal[2] = goalPtr->z;

//        auto planner(std::make_shared<ompl::geometric::RRTstar>(si));
        auto planner(std::make_shared<ompl::geometric::RRTConnect>(si));
//        auto planner(std::make_shared<ompl::geometric::CForest>(si));
//        planner->as<ompl::geometric::CForest>()->setNumThreads(6);
        ss.setPlanner(planner);

        ss.setStartAndGoalStates(start, goal);
        ss.getSpaceInformation()->setStateValidityCheckingResolution(0.01);
        ss.setup();

        bool solved = ss.solve(planTime);

        std::vector<ompl::base::State *> solution;
        if (solved) {
            std::cout << "Found solution:" << std::endl;
            fprintf(stderr, "Found solution:");

            ss.simplifySolution();
            ss.getSolutionPath().print(std::cout);
            std::stringstream sstream;
            ss.getSolutionPath().print(sstream);
//            pubNode.publish(sstream.str());

            og::PathGeometric solPath = ss.getSolutionPath();
            solPath.interpolate(100);

            solution = solPath.getStates();

            *path = new State[solution.size()];
            std::vector<State> pathVec;
            *pathLen = solution.size();
            auto point = path[0];
            for (auto i = 0; i < solution.size(); i++) {
//                auto state3D = solution[i]->as<ob::SE2StateSpace::StateType>();
//                point->x = state3D->getX();
//                point->y = state3D->getY();
//                point->z = state3D->getYaw();

                auto s = solution[i]->as<ob::RealVectorStateSpace::StateType>();
                point->x = s->values[0];
                point->y = s->values[1];
                point->z = s->values[2];
                pathVec.emplace_back(*point);

                point++;
            }

            testStuff(pubNode, pathVec);
        }


        return solved;
    }

private:
    State **path = nullptr;

};

std::intptr_t Init() {
    return (std::intptr_t) new UnityOMPLInterface();
}

void Destroy(std::intptr_t handle) {
    auto ptr = (UnityOMPLInterface *) handle;
    delete ptr;
}

bool RRTSearch(std::intptr_t handle, std::intptr_t goalPtrIn, std::intptr_t statePtrIn,
               FuncCallBack isStateValid, std::intptr_t *pathIn, int *pathLen, double turnRadius, double planTime) {
    auto ptr = (UnityOMPLInterface *) handle;
    auto goalPtr = (State *) goalPtrIn;
    auto statePtr = (State *) statePtrIn;
    auto path = (State **) pathIn;

    return ptr->RRTSearch(goalPtr, statePtr, isStateValid, path, pathLen, turnRadius, planTime);

}



