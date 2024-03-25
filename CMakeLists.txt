cmake_minimum_required(VERSION 3.5)

project(ekf-localization)

include_directories(include)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


# OpenCV
find_package(OpenCV REQUIRED)

add_executable(stereo_VO src/stereo_VO.cpp)

target_link_libraries(stereo_VO ${OpenCV_LIBS})