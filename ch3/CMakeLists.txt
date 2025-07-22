cmake_minimum_required(VERSION 3.24)

project(ch3)

# sudo apt-get install libeigen3-dev
# sudo locate eigen3
find_package(Eigen3 REQUIRED)

add_executable(useGeometry useGeometry.cpp)
target_include_directories(useGeometry PUBLIC ${EIGEN3_INCLUDE_DIRS})

add_executable(useEigen useEigen.cpp)
target_include_directories(useEigen PUBLIC ${EIGEN3_INCLUDE_DIRS})

add_executable(coordinateTransform coordinateTransform.cpp)
target_include_directories(coordinateTransform PUBLIC ${EIGEN3_INCLUDE_DIRS})

add_executable(plotTrajectory plotTrajectory.cpp)
target_include_directories(plotTrajectory PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(plotTrajectory ${Pangolin_LIBRARIES})
find_package(Pangolin REQUIRED)
include_directories(plotTrajectory ${Pangolin_INCLUDE_DIRS})



add_executable(visualizeGeometry visualizeGeometry.cpp)
target_include_directories(visualizeGeometry PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(visualizeGeometry ${Pangolin_LIBRARIES})
find_package(Pangolin REQUIRED)
include_directories(visualizeGeometry ${Pangolin_INCLUDE_DIRS})
