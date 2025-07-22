# cmake_minimum_required(VERSION 2.8)

# project(HelloSLAM)

# add_executable(helloSLAM helloSLAM.cpp)

# add_library(hello_shared SHARED libHelloSLAM.cpp)

# add_executable(useHello useHello.cpp)


# target_link_libraries(useHello hello_shared)

# # sudo apt-get install libeigen3-dev
# # sudo locate eigen3
# find_package(Eigen3 REQUIRED)

# add_executable(useGeometry useGeometry.cpp)

# target_include_directories(useGeometry PUBLIC ${EIGEN3_INCLUDE_DIRS})

cmake_minimum_required(VERSION 2.8)

project(vslam_ws)
find_package(Pangolin REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(OpenGL REQUIRED)
find_package(Sophus REQUIRED)

add_subdirectory(ch2)
add_subdirectory(ch3)
add_subdirectory(ch4)
add_subdirectory(RotationRepresentation)