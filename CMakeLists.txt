CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(PointCloudViewer)
set(CMAKE_CXX_STANDARD 20)

 SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")# C++11 support on GCC
# Find package
find_package(Iridescence REQUIRED)


# Add include dirs and link libraries
add_executable(PointCloudViewer
  main.cpp
)

target_link_libraries(PointCloudViewer  Iridescence::Iridescence)
