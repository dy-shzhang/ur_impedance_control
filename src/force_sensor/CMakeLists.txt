cmake_minimum_required(VERSION 2.8)
project(force_sensor)


if (CMAKE_VERSION VERSION_LESS "3.1")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
	set(CMAKE_CXX_STANDARD 11)
	set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

#include(CheckCXXCompilerFlag)
#CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
#CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
#if(COMPILER_SUPPORTS_CXX11)
#    set(CMAKE_CXX_FLAGS "-std=c++11")
#elseif(COMPILER_SUPPORTS_CXX0X)
#    set(CMAKE_CXX_FLAGS "-std=c++0x")
#else()
#    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler. Suggested solution: update the pkg build-essential ")
#endif()


#SET(LIBRARY_OUTPUT_PATH ${CMAKE_INSTALL_PREFIX}/lib)
find_package(catkin REQUIRED COMPONENTS
  message_generation 
  roscpp
  std_msgs
)
find_package(Boost 1.58 REQUIRED COMPONENTS system thread atomic chrono timer) # system
find_package(Eigen3 REQUIRED)


add_message_files(
  FILES
  Force_Torque.msg
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  roscpp
  std_msgs
)



#include_directories(
#  include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
include_directories(
  include ${catkin_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include/force_sensor)
include_directories(${PROJECT_SOURCE_DIR}/src)




#add_library(FTSensors STATIC)
#target_link_libraries(FTSensors ${Boost_LIBRARIES})
#target_link_libraries(ATI_NetBox_FT_Sensor ${Boost_LIBRARIES}) #  FTSensors

add_library(FTSensors
  ${PROJECT_SOURCE_DIR}/src/FTSensors.cpp
)
target_link_libraries(FTSensors
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)


add_executable(force_torque_pub
  ${PROJECT_SOURCE_DIR}/src/force_torque_pub.cpp
)

target_link_libraries(force_torque_pub
  FTSensors
  ${catkin_LIBRARIES}
)
