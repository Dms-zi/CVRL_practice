cmake_minimum_required(VERSION 3.24)

project(ch8)

# sudo apt-get install libeigen3-dev
# sudo locate eigen3

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(Eigen3 REQUIRED)

add_executable(opencv_optical_flow opencv_optical_flow.cpp)
target_include_directories(opencv_optical_flow PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(opencv_optical_flow ${OpenCV_LIBS})

add_executable(lk_optical_flow lk_optical_flow.cpp)
target_include_directories(lk_optical_flow PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(lk_optical_flow ${OpenCV_LIBS})
target_include_directories(lk_optical_flow PUBLIC ${EIGEN3_INCLUDE_DIRS})




