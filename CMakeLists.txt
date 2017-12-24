project(vis)

SET(execName collect)

CMAKE_MINIMUM_REQUIRED(VERSION 2.4)

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 OLD)
	cmake_policy(SET CMP0015 OLD)  
endif(COMMAND cmake_policy)

SET(EXECUTABLE_OUTPUT_PATH ".")

SET(VERSION_REQ_OCV "3.1")
SET(VERSION_REQ_CUDA "8.0")

#IF(WIN32) # Windows
#    SET(ZED_INCLUDE_DIRS $ENV{ZED_INCLUDE_DIRS})
#    SET(ZED_LIBRARIES $ENV{ZED_LIBRARIES_64})
#    SET(ZED_LIBRARY_DIR $ENV{ZED_LIBRARY_DIR})
#    SET(OPENCV_DIR $ENV{OPENCV_DIR})
#ELSE() # Linux
find_package(ZED 1.0 REQUIRED)
  
    ##For Jetson, OpenCV4Tegra is based on OpenCV2.4
exec_program(uname ARGS -p OUTPUT_VARIABLE CMAKE_SYSTEM_NAME2)
if ( CMAKE_SYSTEM_NAME2 MATCHES "aarch64" OR CMAKE_SYSTEM_NAME2 MATCHES "armv7l" )
    SET(VERSION_REQ_OCV "2.4")
    SET(VERSION_REQ_CUDA "")


    unset(CUDA_USE_STATIC_CUDA_RUNTIME CACHE)
    option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
    message ("CUDA_USE_STATIC_CUDA_RUNTIME : ${CUDA_USE_STATIC_CUDA_RUNTIME}")
       ##to prevent from opencv_dep_cudart dependencies error...
       ## cmake with -DCUDA_USE_STATIC_CUDA_RUNTIME=false can also be called.

endif()
#ENDIF(WIN32)


find_package(OpenCV ${VERSION_REQ_OCV} COMPONENTS core highgui imgproc REQUIRED)
find_package(CUDA ${VERSION_REQ_CUDA} REQUIRED)
#find_package(PCL)
find_package(OpenCV ${VERSION_REQ_OCV} REQUIRED)#for opencv
find_package(CUDA ${VERSION_REQ_CUDA} REQUIRED)#for opencv
find_package(PCL 1.4 REQUIRED)#in thsi version

include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)
#include_directories("${CMAKE_SOURCE_DIR}/src")
include_directories(${VisionWorks-NVXIO_INCLUDE_DIRS})

#include the subdirectory containing our libs
#add_subdirectory (lib)
#include_directories(lib)
#include_directories(lib)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})
#add_library(lib1 ${CMAKE_CURRENT_SOURCE_DIR}/lib/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/lib/*.hpp)

#add_executable (pairwise_incremental_registration pairwise_incremental_registration.cpp)


SET(SRC_FOLDER src)
FILE(GLOB_RECURSE SRC_FILES ${SRC_FOLDER}/*.cpp)
 
ADD_EXECUTABLE(${execName} ${SRC_FILES})

add_definitions(-std=c++0x -g -O3) # -m64)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4") ## needed if you install pcl from repo (and libproj-dev)


add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-std=c++11 -std=gnu++11> $<$<COMPILE_LANGUAGE:C>:-std=c99>)

TARGET_LINK_LIBRARIES(${execName}
                        ${ZED_LIBRARIES}
                        ${OpenCV_LIBRARIES}
                        ${PCL_LIBRARIES}
                        ${CUDA_LIBRARIES} ${CUDA_npps_LIBRARY} ${CUDA_nppi_LIBRARY}
                    )
