#include <ompl/base/Planner.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/util/Time.h>



#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "library.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace std::chrono_literals;


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


class UnityOMPLInterface {
public:
    bool RRTSearch(State *goalPtr, State *statePtr, FuncCallBack isStateValid, State ** path, int * pathLen) {
        initROS();
        MinimalPublisher pubNode;

        ob::RealVectorBounds bounds(2);
        bounds.low[0] = -5;
        bounds.low[1] = -5;
        bounds.high[0] = 5;
        bounds.high[1] = 5;

        auto space(std::make_shared<ob::DubinsStateSpace>(0.1, false));
//        ((ob::StateSpacePtr &) space)->as<ob::SE2StateSpace>()->setBounds(bounds);
        space->setBounds(bounds);

        og::SimpleSetup ss(space);

        auto si = ss.getSpaceInformation();
        si->setStateValidityChecker([=](const ob::State *state) {
            const auto *s = state->as<ob::SE2StateSpace::StateType>();
            statePtr->x = s->getX();
            statePtr->y = s->getY();
            statePtr->z = s->getYaw();

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

        auto planner(std::make_shared<ompl::geometric::RRTstar>(si));
        ss.setPlanner(planner);

        ss.setStartAndGoalStates(start, goal);
        ss.getSpaceInformation()->setStateValidityCheckingResolution(0.01);
        ss.setup();

        bool solved = ss.solve(1.0);

        std::vector<ompl::base::State *> solution;
        if (solved) {
            std::cout << "Found solution:" << std::endl;
            fprintf(stderr, "Found solution:");

            ss.simplifySolution();
            ss.getSolutionPath().print(std::cout);
            std::stringstream sstream;
            ss.getSolutionPath().print(sstream);
            pubNode.publish(sstream.str());

            og::PathGeometric solPath = ss.getSolutionPath();
            solPath.interpolate(1000);
            solution = solPath.getStates();

            *path = new State[solution.size()];
            *pathLen = solution.size();
            auto point = path[0];
            for (auto i = 0; i < solution.size(); i++){
                auto state3D = solution[i]->as<ob::SE2StateSpace::StateType>();
                point->x = state3D->getX();
                point->y = state3D->getY();
                point->z = state3D->getYaw();
//                point->z = state3D->values[2];
                point++;
            }
            point = nullptr;
        }

        return solved;
    }

private:
    State ** path = nullptr;

};

std::intptr_t Init() {
    return (std::intptr_t) new UnityOMPLInterface();
}

void Destroy(std::intptr_t handle) {
    auto ptr = (UnityOMPLInterface *) handle;
    delete ptr;
}

bool RRTSearch(std::intptr_t handle, std::intptr_t goalPtrIn, std::intptr_t statePtrIn,
               FuncCallBack isStateValid, std::intptr_t *pathIn, int* pathLen) {
    auto ptr = (UnityOMPLInterface *) handle;
    auto goalPtr = (State *) goalPtrIn;
    auto statePtr = (State *) statePtrIn;
    auto path = (State **) pathIn;

    return ptr->RRTSearch(goalPtr, statePtr, isStateValid, path, pathLen);

}

