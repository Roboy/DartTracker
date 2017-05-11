project(dart_tracker)
cmake_minimum_required(VERSION 2.8)

if( NOT CMAKE_BUILD_TYPE )
   set( CMAKE_BUILD_TYPE Release )
endif()

add_definitions(-std=c++11 -g)


#==============================================================================
# Dependencies
#==============================================================================
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation rviz tf tf_conversions visualization_msgs dart )
catkin_package(CATKIN_DEPENDS message_runtime dart )

# Qt5
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/../dart/CMakeModules/")

# find packages with modules
find_package(CUDA REQUIRED)

# find packages with pkg-config
find_package(PkgConfig)
pkg_check_modules(eigen3 REQUIRED eigen3)
pkg_check_modules(matheval REQUIRED libmatheval)

find_package(GLEW REQUIRED)

include_directories(
   include
   ${catkin_INCLUDE_DIRS}
   ${Qt5Widgets_INCLUDE_DIRS}
   ${Qt5Core_INCLUDE_DIRS}
   ${Qt5Gui_INCLUDE_DIRS}
   ${eigen3_INCLUDE_DIR}
)

link_libraries(
    ${matheval_LIBRARIES}
)

##########################################################################
#   Build GPU or CPU-only executable depending on cuda                   #
##########################################################################
include_directories( ${CUDA_INCLUDE_DIRS} )
include_directories(${CUDA_TOOLKIT_ROOT_DIR}/samples/common/inc)
cuda_include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(CUDA_NVCC_FLAGS "-arch=sm_35" "--use_fast_math"  "-O3" "--ptxas-options=--verbose") # "-fmad=false" "-DTHRUST_DEVICE_BACKEND=THRUST_DEVICE_BACKEND_OMP"
add_definitions(-DCUDA_BUILD)

set(all_sources ${sources} ${cpu_sources} ${gpu_sources})
link_directories( ${CUDA_TOOLKIT_ROOT_DIR}/lib64/)

qt5_wrap_cpp(MOC_FILES include/dart_tracker/dartTracker.hpp )
cuda_add_library(  dart_tracker src/dartTracker.cpp ${MOC_FILES} )
target_link_libraries( dart_tracker ${QT_LIBRARIES} ${catkin_LIBRARIES}  )