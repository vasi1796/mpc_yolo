cmake_minimum_required(VERSION 3.5)
project(lane_follower)

set(CMAKE_CXX_STANDARD 14)
set(SOURCES src/main.cpp src/utilities.hpp src/MPC.cpp src/MPC.h src/LaneController.h src/LaneController.cpp src/TCPServer.h)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3)
add_definitions(${EIGEN3_DEFINITIONS})

if(NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
    # Possibly map additional variables to the EIGEN3_ prefix.
else()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

include_directories(${aruco_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIR})
include_directories(/usr/local/include)
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIB_DIR})
link_directories(/usr/local/lib)

add_executable(${PROJECT_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME} ipopt ${OpenCV_LIBS} ${Boost_LIBRARIES} -laruco -pthread -lboost_system)