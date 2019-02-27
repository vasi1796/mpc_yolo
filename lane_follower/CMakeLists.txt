cmake_minimum_required(VERSION 3.5)
project(lane_follower)

set(CMAKE_CXX_STANDARD 14)
set(SOURCES src/main.cpp src/utilities.hpp src/MPC.cpp src/MPC.h src/LaneController.h src/LaneController.cpp)

find_package(OpenCV REQUIRED)

include_directories(/usr/local/include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(src/Eigen-3.3)
link_directories(${OpenCV_LIB_DIR})
link_directories(/usr/local/lib)

add_executable(${PROJECT_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME} ipopt ${OpenCV_LIBS})