cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(test_server)

set(SOURCES
    Server.h
    testserver.cpp
)

find_package(Boost REQUIRED)

include_directories(${Boost_INCLUDE_DIR})

add_executable(${PROJECT_NAME} ${SOURCES})
		
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} )
