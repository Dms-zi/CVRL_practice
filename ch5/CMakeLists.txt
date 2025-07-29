cmake_minimum_required(VERSION 3.24)

project(ch5)
set(CMAKE_CXX_FALGS "-std=c++14")
# sudo apt-get install libeigen3-dev
# sudo locate eigen3
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(imageBasics imageBasics.cpp)
target_include_directories(imageBasics PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(imageBasics ${OpenCV_LIBS})


add_executable(undistortImage undistortImage.cpp)
target_include_directories(undistortImage PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(undistortImage ${OpenCV_LIBS})
