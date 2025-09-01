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
    vision/vision.cpp
    main.cpp
)

add_executable(sayatestbench ${SOURCE_FILES})

target_link_libraries(sayatestbench ${DEPENDENCIES} ${OpenCV_LIBS})

install(TARGETS sayatestbench RUNTIME DESTINATION bin)
