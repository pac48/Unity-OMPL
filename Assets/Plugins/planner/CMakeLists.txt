cmake_minimum_required(VERSION 3.22)
project(UnityLib)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/.. CACHE PATH "Installation directory" FORCE)
set(BUILD_SHARED_LIBS ON CACHE PATH "Build shared libraries" FORCE)

# Download and build the repository
include(ExternalProject)
ExternalProject_Add(
        OMPL
        GIT_REPOSITORY https://github.com/ompl/ompl
        GIT_TAG main
        CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
)

include(ExternalProject)
ExternalProject_Add(
        blasfeo
        GIT_REPOSITORY https://github.com/giaf/blasfeo
        GIT_TAG master
        CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=/opt/blasfeo # this is hack, but hpipm requires it?
)

ExternalProject_Add(
        hpipm
        GIT_REPOSITORY https://github.com/giaf/hpipm
        GIT_TAG master
        CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
)

ExternalProject_Add(
        hpipm-cpp
        GIT_REPOSITORY https://github.com/pac48/hpipm-cpp
        GIT_TAG add-option-to-set-cmake-library
        CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} -Dblasfeo_LIBRARY=/opt/blasfeo/lib/libblasfeo.a -Dhpipm_LIBRARY=${CMAKE_INSTALL_PREFIX}/lib/libhpipm.a
)

#set(CMAKE_BUILD_TYPE Debug)
#add_subdirectory(ompl)

find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

add_library(UnityLib SHARED src/library.cpp)
#target_link_libraries(UnityLib ompl2)
target_link_libraries(UnityLib ${CMAKE_INSTALL_PREFIX}/lib/libompl.so)
target_link_libraries(UnityLib ${CMAKE_INSTALL_PREFIX}/lib/libhpipm-cpp.so)
target_link_libraries(UnityLib ${CMAKE_INSTALL_PREFIX}/lib/libhpipm.a)
target_link_libraries(UnityLib /opt/blasfeo/lib/libblasfeo.a)

target_include_directories(UnityLib PRIVATE include)
target_include_directories(UnityLib PRIVATE ${CMAKE_INSTALL_PREFIX}/include/ompl-1.6)
target_include_directories(UnityLib PRIVATE ${CMAKE_INSTALL_PREFIX}/include)

target_include_directories(UnityLib PRIVATE ${builtin_interfaces_INCLUDE_DIRS})
target_include_directories(UnityLib PRIVATE ${libstatistics_collector_INCLUDE_DIRS})
target_include_directories(UnityLib PRIVATE ${rcl_INCLUDE_DIRS})
target_include_directories(UnityLib PRIVATE ${rcl_interfaces_INCLUDE_DIRS})
target_include_directories(UnityLib PRIVATE ${rcl_logging_interface_INCLUDE_DIRS})
target_include_directories(UnityLib PRIVATE ${rcl_logging_spdlog_INCLUDE_DIRS})
target_include_directories(UnityLib PRIVATE ${rcl_yaml_param_parser_INCLUDE_DIRS})
target_include_directories(UnityLib PRIVATE ${rclcpp_INCLUDE_DIRS})
target_include_directories(UnityLib PRIVATE ${rcpputils_INCLUDE_DIRS})
target_include_directories(UnityLib PRIVATE ${rcutils_INCLUDE_DIRS})
target_include_directories(UnityLib PRIVATE ${rmw_INCLUDE_DIRS})
target_include_directories(UnityLib PRIVATE ${rmw_dds_common_INCLUDE_DIRS})
target_include_directories(UnityLib PRIVATE ${rmw_fastrtps_cpp_INCLUDE_DIRS})
target_include_directories(UnityLib PRIVATE ${rmw_fastrtps_shared_cpp_INCLUDE_DIRS})
target_include_directories(UnityLib PRIVATE ${rmw_implementation_INCLUDE_DIRS})
target_include_directories(UnityLib PRIVATE ${rosgraph_msgs_INCLUDE_DIRS})
target_include_directories(UnityLib PRIVATE ${rosidl_default_runtime_INCLUDE_DIRS})
target_include_directories(UnityLib PRIVATE ${rosidl_generator_c_INCLUDE_DIRS})
target_include_directories(UnityLib PRIVATE ${rosidl_runtime_c_INCLUDE_DIRS})
target_include_directories(UnityLib PRIVATE ${rosidl_runtime_cpp_INCLUDE_DIRS})
target_include_directories(UnityLib PRIVATE ${rosidl_typesupport_cpp_INCLUDE_DIRS})
target_include_directories(UnityLib PRIVATE ${rosidl_typesupport_fastrtps_c_INCLUDE_DIRS})
target_include_directories(UnityLib PRIVATE ${rosidl_typesupport_fastrtps_cpp_INCLUDE_DIRS})
target_include_directories(UnityLib PRIVATE ${rosidl_typesupport_interface_INCLUDE_DIRS})
target_include_directories(UnityLib PRIVATE ${rosidl_typesupport_introspection_c_INCLUDE_DIRS})
target_include_directories(UnityLib PRIVATE ${rosidl_typesupport_introspection_cpp_INCLUDE_DIRS})
target_include_directories(UnityLib PRIVATE ${statistics_msgs_INCLUDE_DIRS})
target_include_directories(UnityLib PRIVATE ${std_msgs_INCLUDE_DIRS})
target_include_directories(UnityLib PRIVATE ${tracetools_INCLUDE_DIRS})


target_link_libraries(UnityLib ${rclcpp_LIBRARIES})


# tests
find_package(GTest REQUIRED)
add_executable(tests tests/test.cpp)
target_include_directories(tests PRIVATE include)
target_link_libraries(tests ${GTEST_LIBRARIES} UnityLib)


# install targets
set_target_properties(UnityLib PROPERTIES INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
install(TARGETS UnityLib
        LIBRARY DESTINATION lib
        )


#foreach(lib ${rclcpp_LIBRARIES})
#set(filtered_LIBRARIES ${rclcpp_LIBRARIES})
#list(FILTER filtered_LIBRARIES INCLUDE REGEX "\\.so$")
#
#install(
#        FILES ${filtered_LIBRARIES}
#        LIBRARY DESTINATION lib
#)
#endforeach()

# debug
get_cmake_property(_variableNames VARIABLES)
list(SORT _variableNames)

foreach(_variableName ${_variableNames})
    message(STATUS "${_variableName}=${${_variableName}}")
endforeach()

