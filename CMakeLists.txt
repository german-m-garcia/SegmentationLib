cmake_minimum_required(VERSION 2.6)
project (Segmentation)

#set(OpenCV_DIR "~/workspace/opencv/build")
#To build the project openCV is required
find_package( OpenCV 2.4 REQUIRED ml core gpu highgui)
if(OpenCV_FOUND)
	message("OPENCV FOUND")
	MESSAGE( STATUS ${OpenCV_DIR})
	MESSAGE( STATUS ${OpenCV_INCLUDE_DIRS})
endif()
message("OpenCV_LIBS (2.4?) = ${OpenCV_LIBS}")

set(PCL_DIR "~/workspace/pcl/build/")
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Boost REQUIRED program_options)

add_definitions(-DHAVE_STRING_H)


SET(CMAKE_CXX_FLAGS "-O4 -std=c++11 -g -Wall")
#SET(CMAKE_CXX_FLAGS "-O4 -std=c++0x -g -fpic")
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}//src)
set(PROJECT_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)


#Include Required Directories
include_directories("${PROJECT_INCLUDE_DIR}")
include_directories("${PROJECT_SOURCE_DIR}")
include_directories("${OpenCV_INCLUDE_DIRS}")


FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
	message("OPENMP FOUND")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()


# Create a shared library
#add_library(SEGMENTATION
#${PROJECT_SOURCE_DIR}/segmentation.cpp
#)

add_library(SEGMENTATION SHARED
   ${CMAKE_CURRENT_SOURCE_DIR}/src/segmentation.cpp
   
   ${CMAKE_CURRENT_SOURCE_DIR}/src/objects/svm_wrapper.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/objects/ObjectDetector.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/pcl_segmentation.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/base_segmentation.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/segment.cpp
   #${CMAKE_CURRENT_SOURCE_DIR}/src/z_adaptive_normals.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/surface_cluster.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/epbase.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/src/cluster_normals_to_planes.cpp    
   ${CMAKE_CURRENT_SOURCE_DIR}/src/utils.cpp  
   #remove if using the latest PCL 
   ${CMAKE_CURRENT_SOURCE_DIR}/src/lccp_segmentation.cpp
 )

 


#Create an Executable for segment reader
add_executable(segment ${PROJECT_SOURCE_DIR}/segmenter.cpp)
target_link_libraries( segment SEGMENTATION  ${Boost_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(ms_segment ${PROJECT_SOURCE_DIR}/ms_segmenter.cpp)
target_link_libraries( ms_segment  SEGMENTATION ${Boost_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} )


