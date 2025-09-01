cmake_minimum_required(VERSION 3.11)

project(sayatestbench)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")

find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(EIGEN3 REQUIRED eigen3)
  set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
endif()

find_package( OpenCV REQUIRED )
set(DEPENDENCIES ${DEPENDENCIES} realsense2)

include_directories("/usr/local/include")
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories( ${OpenCV_INCLUDE_DIRS} )

#add_subdirectory(LocalMap)
#add_subdirectory(vision)

set(SOURCE_FILES
    SayaGlobals/globals.cpp
    vision/vision.cpp
    LocalMap/grid_map/src/GridMap.cpp
    LocalMap/grid_map/src/GridMapMath.cpp
    LocalMap/grid_map/src/SubmapGeometry.cpp
    LocalMap/grid_map/src/BufferRegion.cpp
    LocalMap/grid_map/src/Polygon.cpp
    LocalMap/grid_map/src/CubicInterpolation.cpp
    LocalMap/grid_map/src/iterators/GridMapIterator.cpp
    LocalMap/grid_map/src/iterators/SubmapIterator.cpp
    LocalMap/grid_map/src/iterators/CircleIterator.cpp
    LocalMap/grid_map/src/iterators/EllipseIterator.cpp
    LocalMap/grid_map/src/iterators/SpiralIterator.cpp
    LocalMap/grid_map/src/iterators/PolygonIterator.cpp
    LocalMap/grid_map/src/iterators/LineIterator.cpp
    LocalMap/locmap.cpp
    main.cpp
)

add_executable(sayatestbench ${SOURCE_FILES})

target_link_libraries(sayatestbench ${DEPENDENCIES} ${OpenCV_LIBS})

install(TARGETS sayatestbench RUNTIME DESTINATION bin)
