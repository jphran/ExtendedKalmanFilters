cmake_minimum_required(VERSION 3.17)
project(ExtendedKalmanFilters)

set(CMAKE_CXX_STANDARD 20)

include_directories(/usr/local/include
        ${PROJECT_SOURCE_DIR}/include)

#### Calculate Jacobian to linearize measurment function that converts sensor space into state space
add_executable(CalculateJacobian src/CalculateJacobian/main.cpp)

#### 2-D kalman filter with 1 sensor
add_executable(KalmanFilterTwoDimOneSensor
        src/KalmanFilterEqnsTwoDimOneSensor/main.cpp
        src/KalmanFilterEqnsTwoDimOneSensor/kalman_filter.cpp
        include/KalmanFilterEqnsTwoDimOneSensor/kalman_filter.h
        src/KalmanFilterEqnsTwoDimOneSensor/tracking.cpp
        include/KalmanFilterEqnsTwoDimOneSensor/tracking.h
        include/KalmanFilterEqnsTwoDimOneSensor/measurement_package.h)

#### 1-D kalman filter
add_executable(KalmanFilterEqnsOneDimension src/KalmanFilterEqnsOneDimension/KalmanFilterEqnsOneDimension.cpp)