cmake_minimum_required(VERSION 3.24)
project(UnityLib)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/.. CACHE PATH "Installation directory" FORCE)

# Download and build the repository
include(ExternalProject)
ExternalProject_Add(
        OMPL
        GIT_REPOSITORY https://github.com/ompl/ompl
        GIT_TAG main
        CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})


add_library(UnityLib SHARED library.cpp)
target_link_libraries(UnityLib ${CMAKE_INSTALL_PREFIX}/lib/libompl.so)
target_include_directories(UnityLib PRIVATE ${CMAKE_INSTALL_PREFIX}/include/ompl-1.6)

install(TARGETS UnityLib
        LIBRARY DESTINATION lib
        )
