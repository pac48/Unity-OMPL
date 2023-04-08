#include <ompl/base/Planner.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/goals/GoalState.h>

#include "library.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;


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

void RRTSearch(std::intptr_t goalPtrIn, std::intptr_t statePtrIn, FuncCallBack isStateValid) {
  // the value in the statePtrIn is used to check in isStateValid function.
  auto goalPtr = (Goal *) goalPtrIn;
  auto statePtr = (State *) statePtrIn;



//  auto spaceX = std::make_shared<ob::RealVectorStateSpace>(1);
//  auto spaceY = std::make_shared<ob::RealVectorStateSpace>(1);
//  auto spaceZ = std::make_shared<ob::RealVectorStateSpace>(1);
//  spaceX->setBounds(-5.0, 5.0);
//  spaceY->setBounds(-5.0, 5.0);
//  spaceZ->setBounds(-0.5, 0.5);
//  auto space = spaceX * spaceY * spaceZ;
  auto space = std::make_shared<ob::RealVectorStateSpace>(3);
  space->setBounds(-5.0, 5.0);

  og::SimpleSetup ss(space);
  ss.setStateValidityChecker([=](const ob::State *state) {
    const auto* state3D = state->as<ob::RealVectorStateSpace::StateType>();
    statePtr->x = state3D->values[0];
    statePtr->y = state3D->values[1];
    statePtr->z = state3D->values[2];
    return isStateValid();
  });

  ob::ScopedState<> start(space);
  start[0] = statePtr->x;
  start[1] = statePtr->y;
  start[2] = statePtr->z;

  ob::ScopedState<> goal(space);
  start[0] = goalPtr->x;
  start[1] = goalPtr->y;
  start[2] = goalPtr->z;

  ss.setStartAndGoalStates(start, goal);

  ob::PlannerStatus solved = ss.solve(1.0);

  std::vector<ompl::base::State*> solution;
  if (solved)
  {
    std::cout << "Found solution:" << std::endl;
    ss.simplifySolution();
    ss.getSolutionPath().print(std::cout);

    solution = ss.getSolutionPath().getStates();
  }


}
