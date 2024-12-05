CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(PointCloudViewer)
set(CMAKE_CXX_STANDARD 20)

 SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++2a ")# C++20 support on GCC
# Find package
find_package(Iridescence REQUIRED)
find_package(PDAL 2.0.0 REQUIRED CONFIG)


# Add include dirs and link libraries
add_executable(PointCloudViewer
  main.cpp
)

target_link_libraries(PointCloudViewer  Iridescence::Iridescence)

target_link_libraries(PointCloudViewer PRIVATE ${PDAL_LIBRARIES})
target_include_directories(PointCloudViewer PRIVATE
    ${PDAL_INCLUDE_DIRS}
    ${PDAL_INCLUDE_DIRS}/pdal)

