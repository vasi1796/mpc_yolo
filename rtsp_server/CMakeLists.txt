cmake_minimum_required(VERSION 2.8)
project(H264LiveStreamer)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)
include_directories(~/live/liveMedia/include)
include_directories(~/live/groupsock/include)
include_directories(~/live/BasicUsageEnvironment/include)
include_directories(~/live/UsageEnvironment/include)
include_directories(~/ffmpeg_build/include/libswscale)
include_directories(~/ffmpeg_build/include/libswresample)
include_directories(~/ffmpeg_build/include)
link_directories(~/ffmpeg_build/lib)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})
set(LIVE_LIBRARIES liveMedia groupsock BasicUsageEnvironment UsageEnvironment)

set(H264LiveStreamer
        H264LiveServerMediaSession.h
        H264LiveServerMediaSession.cxx
        H264LiveStreamer.cpp
        x264Encoder.cxx
        x264encoder.h
        LiveSourceWithx264.h
        LiveSourceWithx264.cxx)

add_executable(${PROJECT_NAME} ${H264LiveStreamer})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME}  ${LIVE_LIBRARIES})

target_link_libraries(${PROJECT_NAME} libx264.a -lpthread -ldl)
target_link_libraries(${PROJECT_NAME} libswscale.a)
target_link_libraries(${PROJECT_NAME} libavutil.a)
target_link_libraries(${PROJECT_NAME} ${BOOST_LIBRARIES} -lboost_system -lboost_thread)

