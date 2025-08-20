cmake_minimum_required(VERSION 3.24)

project(ch6)

# sudo apt-get install libeigen3-dev
# sudo locate eigen3
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(g2o REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})
find_package(Ceres REQUIRED)



add_executable(gaussNewton gaussNewton.cpp)
target_include_directories(gaussNewton PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_include_directories(gaussNewton PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(gaussNewton ${OpenCV_LIBS})



add_executable(g2oCurveFitting g2oCurveFitting.cpp)
target_include_directories(g2oCurveFitting PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_include_directories(g2oCurveFitting PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(g2oCurveFitting
    g2o_core
    g2o_stuff
    ${OpenCV_LIBS}
    -lfmt
    -lspdlog       
)
add_executable(ceresCurveFitting ceresCurveFitting.cpp)
target_include_directories(ceresCurveFitting PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_include_directories(ceresCurveFitting PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(ceresCurveFitting
    ${CERES_LIBRARIES}
    ${OpenCV_LIBS}
      
)