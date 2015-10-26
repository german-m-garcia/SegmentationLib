cmake_minimum_required(VERSION 2.6)
project (Segmentation)

set(OpenCV_DIR "~/workspace/opencv/build")
#To build the project openCV is required
find_package( OpenCV REQUIRED)

if(OpenCV_FOUND)
	message("OPENCV FOUND")
	MESSAGE( STATUS ${OpenCV_DIR})
endif()


add_definitions(-DHAVE_STRING_H)


SET(CMAKE_CXX_FLAGS "-O4 -std=c++0x -g -fpic")
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(PROJECT_SOURCE_DIR ${CMAKE_SOURCE_DIR}/src)
set(PROJECT_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)





#Include Required Directories
include_directories("${PROJECT_INCLUDE_DIR}")
include_directories("${PROJECT_SOURCE_DIR}")
include_directories(${OpenCV_INCLUDE_DIRS})


FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
	message("OPENMP FOUND")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()


# Create a shared library
add_library(SEGMENTATION
${PROJECT_SOURCE_DIR}/Segmentation.cpp

)





#Create an Executable for segment reader
add_executable(segment ${PROJECT_SOURCE_DIR}/segmenter.cpp)
target_link_libraries( segment SEGMENTATION  ${OpenCV_LIBS} )

add_executable(ms_segment ${PROJECT_SOURCE_DIR}/ms_segmenter.cpp)
target_link_libraries( ms_segment SEGMENTATION  ${OpenCV_LIBS} )


