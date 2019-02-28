cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(webcam)

set(SOURCES
    src/yolo_console_dll.cpp
    src/yolo_v2_class.hpp
    src/darknet.h
)
# create a list of files to copy
set( THIRD_PARTY_DLLS
   ${CMAKE_SOURCE_DIR}/bin/libvlc.dll
   ${CMAKE_SOURCE_DIR}/bin/libvlccore.dll
   ${CMAKE_SOURCE_DIR}/bin/yolo_cpp_dll.dll
   ${CMAKE_SOURCE_DIR}/bin/pthreadVC2.dll
   ${CMAKE_SOURCE_DIR}/libgcc_s_seh-1.dll
   ${CMAKE_SOURCE_DIR}/libstdc++-6.dll
)

find_package(OpenCV)
find_package(CUDA REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(src/vlc_sdk/include)

link_directories(${OpenCV_LIB_DIR})
link_directories(src/vlc_sdk/lib)

add_executable(${PROJECT_NAME} ${SOURCES})
		
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} "${CMAKE_SOURCE_DIR}/lib/yolo_cpp_dll.lib" libvlc.lib libvlccore.lib)

add_custom_target(copy-runtime-files ALL
    COMMAND cmake -E copy_directory "${CMAKE_SOURCE_DIR}/bin" "${CMAKE_BINARY_DIR}/$<CONFIG>/"
    DEPENDS ${MY_TARGET})