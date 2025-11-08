cmake_minimum_required(VERSION 3.12)

project( KinectTouch )

file(GLOB_RECURSE OSC
  src/oscpack/osc/*.h
  src/oscpack/osc/*.cpp
  # "src/oscpack/osc/*/*.h"
  src/oscpack/ip/*.cpp
  src/oscpack/ip/*.h
  "src/oscpack/osc/ip/posix/*.cpp"
)

foreach(file ${OSC})
  message(STATUS "OSC file: ${file}")
endforeach()

file(GLOB TUIO
  "src/TUIO/*.h"
  "src/TUIO/*.cpp"
)

find_package( OpenCV REQUIRED )
find_package(Threads REQUIRED)

add_executable( KinectTouch src/KinectTouch.cpp ${OSC} ${TUIO})
#add_executable( KinectTouch src/KinectTouch.cpp)

include_directories(
  "/usr/include/ni"
  "src/TUIO"
  "src/oscpack"
)

target_link_libraries( KinectTouch X11 Xtst ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT} OpenNI )
