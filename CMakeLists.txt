cmake_minimum_required(VERSION 3.16)
project(rkd)

set(CMAKE_CXX_STANDARD 17)

add_definitions(-DPROJECT_DIR="${CMAKE_SOURCE_DIR}")

# eigen
find_package(Eigen3 CONFIG REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# eigenpy
find_package(eigenpy CONFIG REQUIRED)
include_directories(${EIGENPY_INCLUDE_DIRS})

# pinocchio
find_package(pinocchio CONFIG REQUIRED)
include_directories(${PINOCCHIO_INCLUDE_DIRS})

# set include directories
include_directories(include)

# generate library
add_library(robot_kinematics_dynamics STATIC src/robot_kinematics_dynamics.cpp)

# install library and headers
install(TARGETS robot_kinematics_dynamics ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

# tests
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

