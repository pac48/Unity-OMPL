#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include "library.h"

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

void RRTSearch(std::intptr_t goalPtrIn, std::intptr_t statePtrIn, FuncCallBack cb) {
    auto goalPtr = (Goal*) goalPtrIn;
    auto statePtr = (State *) statePtrIn;


    for (auto i = 0; i < 3; i++) {
        cb();
    }

}
