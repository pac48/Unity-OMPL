#include <ompl/base/Planner.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/goals/GoalState.h>

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "library.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace std::chrono_literals;


struct Object {
    float x;
    float y;
};
struct Goal {
    float x;
    float y;
    float z;
};

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
    bool RRTSearch(Goal *goalPtr, State *statePtr, FuncCallBack isStateValid, State ** path, int * pathLen) {
        initROS();
        MinimalPublisher pubNode;

        auto space = std::make_shared<ob::RealVectorStateSpace>(2);
        space->setBounds(-5.0, 5.0);

        og::SimpleSetup ss(space);
        ss.setStateValidityChecker([=](const ob::State *state) {
            const auto *state3D = state->as<ob::RealVectorStateSpace::StateType>();
            statePtr->x = state3D->values[0];
            statePtr->y = state3D->values[1];
//        statePtr->z = state3D->values[2];
            return isStateValid();
        });

        ob::ScopedState<> start(space);
        start[0] = statePtr->x;
        start[1] = statePtr->y;
//    start[2] = statePtr->z;

        ob::ScopedState<> goal(space);
        goal[0] = goalPtr->x;
        goal[1] = goalPtr->y;
//    goal[2] = goalPtr->z;

        ss.setStartAndGoalStates(start, goal);

        ob::PlannerStatus solved = ss.solve(1.0);

        std::vector<ompl::base::State *> solution;
        if (solved) {
            std::cout << "Found solution:" << std::endl;
            fprintf(stderr, "Found solution:");
            ss.simplifySolution();
            ss.getSolutionPath().print(std::cout);
            std::stringstream sstream;
            ss.getSolutionPath().print(sstream);
            pubNode.publish(sstream.str());

            solution = ss.getSolutionPath().getStates();

            *path = new State[solution.size()];
            *pathLen = solution.size();
            auto point = path[0];
            for (auto i = 0; i < solution.size(); i++){
                auto state3D = solution[i]->as<ob::RealVectorStateSpace::StateType>();
                point->x = state3D->values[0];
                point->y = state3D->values[1];
                point->z = 0;
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
    auto goalPtr = (Goal *) goalPtrIn;
    auto statePtr = (State *) statePtrIn;
    auto path = (State **) pathIn;

    return ptr->RRTSearch(goalPtr, statePtr, isStateValid, path, pathLen);

}

