cmake_minimum_required(VERSION 2.6)
project (Segmentation)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/Modules/" ${CMAKE_MODULE_PATH})


#set(OpenCV_DIR "~/workspace/opencv/build")
#To build the project openCV is required
find_package( OpenCV 2.4 REQUIRED ml core gpu highgui)
if(OpenCV_FOUND)
	message("OPENCV FOUND")
	MESSAGE( STATUS ${OpenCV_DIR})
	MESSAGE( STATUS ${OpenCV_INCLUDE_DIRS})
endif()
message("OpenCV_LIBS (2.4?) = ${OpenCV_LIBS}")

find_package(Qt4 COMPONENTS Widgets Declarative)

#PCL
#set(PCL_DIR "~/workspace/pcl/build/")
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(CUDA REQUIRED) 

find_package(Boost REQUIRED program_options)

add_definitions(-DHAVE_STRING_H)

# OctreeLib required
#find_package(OctreeLib REQUIRED)
#include_directories(${OCTREELIB_INCLUDE_DIRS})

# G2O required
#find_package(G2O REQUIRED)
#include_directories(${G2O_INCLUDE_DIR})

# GSL required
#find_package(GSL REQUIRED)
#include_directories(${GSL_INCLUDE_DIRS})
#link_directories(${GSL_LIBRARY_DIRS})

# TBB required
#find_package(TBB REQUIRED)
#include_directories(${TBB_INCLUDE_DIRS})

# For building the CHOLMOD / CSPARSE solvers
#find_package(Cholmod)
#find_package(BLAS)
#find_package(LAPACK)
#find_package(CSparse)
#include_directories(${CSPARSE_INCLUDE_DIR})


#MRSmap
#set(MRSmap_LIB_PATH /home/martin/workspace/mrsmap/)
#ADD_LIBRARY(mrsmapLib SHARED IMPORTED)
#SET_TARGET_PROPERTIES(mrsmapLib PROPERTIES
    #IMPORTED_LOCATION ${MRSmap_LIB_PATH}/build/devel/lib/libmrsmap.so)
#include_directories(${MRSmap_LIB_PATH}/include)

#MRSSlam
#set(MRSslam_LIB_PATH /home/martin/workspace/mrsmap/)
#ADD_LIBRARY(mrsslamLib SHARED IMPORTED)
#SET_TARGET_PROPERTIES(mrsslamLib PROPERTIES
    #IMPORTED_LOCATION ${MRSslam_LIB_PATH}/build/devel/lib/libmrsslam.so)
#include_directories(${MRSslam_LIB_PATH}/include)




SET(CMAKE_CXX_FLAGS "-O4 -std=c++11 -g -Wall -fopenmp")
#SET(CMAKE_CXX_FLAGS "-O4 -std=c++0x -g -fpic")
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
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
   ${PROJECT_SOURCE_DIR}/segmentation.cpp
   ${PROJECT_SOURCE_DIR}/ms_segmentation.cpp
   
   ${PROJECT_SOURCE_DIR}/objects/svm_wrapper.cpp
   ${PROJECT_SOURCE_DIR}/objects/ObjectDetector.cpp
   ${PROJECT_SOURCE_DIR}/objects/Training.cpp
   ${PROJECT_SOURCE_DIR}/objects/MRSMapWrap.cpp
   
   ${PROJECT_SOURCE_DIR}/thinning/thinner.cpp
   
   ${PROJECT_SOURCE_DIR}/pcl_segmentation.cpp
   ${PROJECT_SOURCE_DIR}/base_segmentation.cpp
   ${PROJECT_SOURCE_DIR}/segment.cpp
 #  ${PROJECT_SOURCE_DIR}/z_adaptive_normals.cpp
   ${PROJECT_SOURCE_DIR}/surface_cluster.cpp
   ${PROJECT_SOURCE_DIR}/epbase.cpp
   ${PROJECT_SOURCE_DIR}/cluster_normals_to_planes.cpp    
   ${PROJECT_SOURCE_DIR}/utils.cpp
   #${PROJECT_SOURCE_DIR}/clahe.cpp  
   #hack to work with PCL 1.7
   ${PROJECT_SOURCE_DIR}/pcl_hack/uniform_sampling.cpp
   
   ${PROJECT_SOURCE_DIR}/contours/Contours.cpp
   ${PROJECT_SOURCE_DIR}/contours/Gestalts.cpp
   
   #${PROJECT_SOURCE_DIR}/features/test_features.cpp
     
   
   #remove if using the latest PCL 
   #${PROJECT_SOURCE_DIR}/lccp_segmentation.cpp
 )


#Create an Executable for testing contours
add_executable(contours ${PROJECT_SOURCE_DIR}/apps/test_contours.cpp)
target_link_libraries( contours SEGMENTATION  ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

#Create an Executable for testing contours
add_executable(harris ${PROJECT_SOURCE_DIR}/apps/test_harris.cpp)
target_link_libraries( harris SEGMENTATION  ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS}) 



#Create an Executable for segment reader
add_executable(segment ${PROJECT_SOURCE_DIR}/segmenter.cpp)
target_link_libraries( segment SEGMENTATION  ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS}) #mrsmapLib mrsslamLib)

#Create an Executable for test main file
add_executable(test ${PROJECT_SOURCE_DIR}/test.cpp)
target_link_libraries( test SEGMENTATION  ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

#Create an Executable for test_mesh main file
add_executable(mesh ${PROJECT_SOURCE_DIR}/mesh/test_mesh.cpp)
target_link_libraries( mesh SEGMENTATION  ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(render ${PROJECT_SOURCE_DIR}/mesh/render_mesh.cpp)
target_link_libraries( render SEGMENTATION  ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

#add_executable(tessel ${PROJECT_SOURCE_DIR}/mesh/tessel.cpp)
#target_link_libraries( tessel SEGMENTATION  ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})


add_executable(ms_segment ${PROJECT_SOURCE_DIR}/ms_segmenter.cpp)
target_link_libraries( ms_segment  SEGMENTATION ${Boost_LIBRARIES}  ${OpenCV_LIBS}  ${PCL_LIBRARIES} )

add_executable(clahe ${PROJECT_SOURCE_DIR}/clahe.cpp)
target_link_libraries( clahe  SEGMENTATION ${Boost_LIBRARIES}  ${OpenCV_LIBS}  ${PCL_LIBRARIES} )


add_executable(features ${PROJECT_SOURCE_DIR}/features/test_features.cpp)
target_link_libraries( features  SEGMENTATION ${Boost_LIBRARIES}  ${OpenCV_LIBS}  ${PCL_LIBRARIES} )

#add_executable(interest ${PROJECT_SOURCE_DIR}/features/interest_points.cpp)
#target_link_libraries( interest  SEGMENTATION ${Boost_LIBRARIES}  ${OpenCV_LIBS}  ${PCL_LIBRARIES} )


#add_executable(tomasi ${PROJECT_SOURCE_DIR}/features/tomasi.cpp)
#target_link_libraries( tomasi  SEGMENTATION ${Boost_LIBRARIES}  ${OpenCV_LIBS}  ${PCL_LIBRARIES} )
