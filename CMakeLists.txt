cmake_minimum_required(VERSION 3.5)
project(ekf-localization)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(stereo_VO src/stereo_VO.cpp)

target_link_libraries(stereo_VO ${OpenCV_LIBS})