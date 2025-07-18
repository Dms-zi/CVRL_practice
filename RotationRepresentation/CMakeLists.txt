cmake_minimum_required(VERSION 3.24)


project(RotationRepresentation)

# sudo apt-get install libeigen3-dev
# sudo locate eigen3
find_package(Eigen3 REQUIRED)


add_executable(main main.cpp )
target_include_directories(main PUBLIC ${EIGEN3_INCLUDE_DIRS})

