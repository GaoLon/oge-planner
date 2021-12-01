cmake_minimum_required(VERSION 2.8.3)
project(trat_map)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  cv_bridge
  message_filters
)

find_package(Eigen3 REQUIRED)
# find_package(PCL 1.7 REQUIRED)
find_package(PCL REQUIRED)

set(PCL_INCLUDE_DIRS ${PCL_INCLUDE_DIR})

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp std_msgs
 LIBRARIES trat_map
 DEPENDS PCL
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

include_directories("/usr/include/pcl-1.8")

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library( trat_map 
    src/trat_map.cpp
    )
target_link_libraries( trat_map
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    )  
install(TARGETS trat_map
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

