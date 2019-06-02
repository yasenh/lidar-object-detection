cmake_minimum_required(VERSION 2.8)
project(lidar-object-detection)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

# Try to find Point Cloud Library
find_package(PCL 1.2 REQUIRED)
if (PCL_FOUND)
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
else ()
    message(FATAL_ERROR "Could not locate PCL")
endif()

file(GLOB SOURCE_FILES src/*.cpp src/render/*.cpp)

add_executable(${CMAKE_PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries (${CMAKE_PROJECT_NAME} ${PCL_LIBRARIES})




