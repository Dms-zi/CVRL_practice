cmake_minimum_required(VERSION 3.24)

project(ReRun)

# sudo apt-get install libeigen3-dev
# sudo locate eigen3
find_package(Eigen3 REQUIRED)


add_executable(tutorial tutorial.cpp)
target_include_directories(tutorial PUBLIC ${EIGEN3_INCLUDE_DIRS})

add_executable(plotTrajectory plotTrajectory.cpp)
target_include_directories(plotTrajectory PUBLIC ${EIGEN3_INCLUDE_DIRS})

add_executable(plotArrow plotArrow.cpp)
target_include_directories(plotArrow PUBLIC ${EIGEN3_INCLUDE_DIRS})

# Download the rerun_sdk
include(FetchContent)
FetchContent_Declare(rerun_sdk URL
    https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)

# Link against rerun_sdk.
target_link_libraries(tutorial PRIVATE rerun_sdk)
target_link_libraries(plotTrajectory PRIVATE rerun_sdk)
target_link_libraries(plotArrow  PRIVATE rerun_sdk)