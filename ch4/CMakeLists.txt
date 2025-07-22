cmake_minimum_required(VERSION 3.24)

project(ch4)

# sudo apt-get install libeigen3-dev
# sudo locate eigen3
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)

add_executable(useSophus useSophus.cpp)
target_include_directories(useSophus PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(useSophus Sophus::Sophus)


add_executable(trajectoryError trajectoryError.cpp)

target_include_directories(trajectoryError PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(trajectoryError  ${Pangolin_LIBRARIES} ${FMT_LIBRARIES})
# add_executable(useEigen useEgen.cpp)
# target_include_directories(useEigen PUBLIC ${EIGEN3_INCLUDE_DIRS})

# add_executable(coordinateTransform coordinateTransform.cpp)
# target_include_directories(coordinateTransform PUBLIC ${EIGEN3_INCLUDE_DIRS})

# add_executable(plotTrajectory plotTrajectory.cpp)
# target_include_directories(plotTrajectory PUBLIC ${EIGEN3_INCLUDE_DIRS})
# target_link_libraries(plotTrajectory ${Pangolin_LIBRARIES})
# find_package(Pangolin REQUIRED)
# include_directories(plotTrajectory ${Pangolin_INCLUDE_DIRS})
