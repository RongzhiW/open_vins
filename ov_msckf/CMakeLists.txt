cmake_minimum_required(VERSION 2.8.8)

# Project name
project(ov_msckf)

message(STATUS "project name:"${PROJECT_NAME})
message(STATUS "project bin:"${PROJECT_BINARY_DIR})
message(STATUS "devel path:"${CATKIN_DEVEL_PREFIX})
message(STATUS "bin path:"${EXECUTABLE_OUTPUT_PATH})
message(STATUS "lib path:"${LIBRARY_OUTPUT_PATH})
set(EXECUTABLE_OUTPUT_PATH ${CATKIN_DEVEL_PREFIX}/${PROJECT_NAME})
# Include our cmake files
#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/)

# Find catkin (the ROS build system)
#find_package(catkin REQUIRED COMPONENTS roscpp rosbag tf std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs cv_bridge ov_core)
#find_package(catkin REQUIRED COMPONENTS roscpp rosbag tf std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs ov_core)
find_package(catkin REQUIRED COMPONENTS roscpp rosbag tf std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs)

# Include libraries
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)

# display message to user
message(STATUS "EIGEN VERSION: " ${EIGEN3_VERSION})
message(STATUS "BOOST VERSION: " ${Boost_VERSION})

# Describe catkin project
catkin_package()

# Try to compile with c++11
# http://stackoverflow.com/a/25836953
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Enable compile optimizations
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Enable debug flags (use if you want to debug in gdb)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized -Wmaybe-uninitialized")

set(OpenCV_DIR "/usr/local/share/OpenCV")
find_package(OpenCV 3 REQUIRED)

message(STATUS "OPENCV VERSION: " ${OpenCV_VERSION})
message(STATUS "opencv:"${OpenCV_INCLUDE_DIRS})
find_package(cv_bridge REQUIRED)
message(STATUS "cv_bridge include:${cv_bridge_INCLUDE_DIRS}")
message(STATUS "cv_bridge lib:${cv_bridge_LIBRARIES}")

# Include our header files
include_directories(
        src
        ../ov_core/src
        ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${cv_bridge_INCLUDE_DIRS}
)


# Set link libraries used by all binaries
list(APPEND thirdparty_libraries
        ${Boost_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES}
        ${cv_bridge_LIBRARIES}
)

message(STATUS "catkin lib:"${catkin_LIBRARIES})

##################################################
# Make the core library
##################################################
#add_library(ov_msckf_lib
#        src/state/State.cpp
#        src/state/StateHelper.cpp
#        src/state/Propagator.cpp
#        src/core/VioManager.cpp
#        src/core/RosVisualizer.cpp
#        src/update/UpdaterHelper.cpp
#        src/update/UpdaterMSCKF.cpp
#        src/update/UpdaterSLAM.cpp
#)
#target_link_libraries(ov_msckf_lib ${thirdparty_libraries})
set(ov_msckf_src
        src/state/State.cpp
        src/state/StateHelper.cpp
        src/state/Propagator.cpp
        src/core/VioManager.cpp
        src/core/RosVisualizer.cpp
        src/update/UpdaterHelper.cpp
        src/update/UpdaterMSCKF.cpp
        src/update/UpdaterSLAM.cpp
        src/core/camera_model_utils.cpp src/core/camera_model_utils.h)
set(ov_core_src
        ../ov_core/src/dummy.cpp
        ../ov_core/src/init/InertialInitializer.cpp
        ../ov_core/src/sim/BsplineSE3.cpp
        ../ov_core/src/sim/Simulator.cpp
        ../ov_core/src/track/TrackBase.cpp
        #        src/track/TrackAruco.cpp
        #        src/track/TrackDescriptor.cpp
        ../ov_core/src/track/TrackKLT.cpp
        ../ov_core/src/track/TrackSIM.cpp
        ../ov_core/src/types/Landmark.cpp
        ../ov_core/src/feat/Feature.cpp
        ../ov_core/src/feat/FeatureInitializer.cpp
        )


##################################################
# Make binary files!
##################################################
add_executable(run_serial_msckf src/run_serial_msckf.cpp ${ov_msckf_src} ${ov_core_src})
target_link_libraries(run_serial_msckf ${thirdparty_libraries})

#add_executable(run_simulation src/run_simulation.cpp)
#target_link_libraries(run_simulation ov_msckf_lib ${thirdparty_libraries})
#
#add_executable(run_subscribe_msckf src/run_subscribe_msckf.cpp)
#target_link_libraries(run_subscribe_msckf ov_msckf_lib ${thirdparty_libraries})
