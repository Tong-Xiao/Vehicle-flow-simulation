cmake_minimum_required(VERSION 2.8.3)
project(multi_car)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  gazebo_ros_control
  geometry_msgs
  position_controllers
  roscpp
  rospy
  sicktoolbox
  sicktoolbox_wrapper
  std_msgs
  tf
  transmission_interface
  velocity_controllers
  velodyne_pointcloud
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

# Need to have these in order to find/include boost stuff
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS}
# ${catkin_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(src)

catkin_package(
  INCLUDE_DIRS src
  LIBRARIES multi_car
  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs velodyne_pointcloud tf
)

include_directories(multi_car/src)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#ambulance
add_library(ambulancegazebo SHARED src/cont.cc src/cont.hh)
target_link_libraries(ambulancegazebo ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ambulancegazebo
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#bus
add_library(busgazebo SHARED src/cont.cc src/cont.hh)
target_link_libraries(busgazebo ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS busgazebo
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#pickup
add_library(pickupgazebo SHARED src/cont.cc src/cont.hh)
target_link_libraries(pickupgazebo ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS pickupgazebo
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#fire_truck
add_library(fire_truckgazebo SHARED src/cont.cc src/cont.hh)
target_link_libraries(fire_truckgazebo ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS fire_truckgazebo
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#prius
add_library(priusgazebo SHARED src/cont.cc src/cont.hh)
target_link_libraries(priusgazebo ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS priusgazebo
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#suv
add_library(suvgazebo SHARED src/cont.cc src/cont.hh)
target_link_libraries(suvgazebo ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS suvgazebo
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#suv_1
add_library(suv_1gazebo SHARED src/cont.cc src/cont.hh)
target_link_libraries(suv_1gazebo ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS suv_1gazebo
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#ambulance_1(add car example step1)
add_library(ambulance_1gazebo SHARED src/cont.cc src/cont.hh)
target_link_libraries(ambulance_1gazebo ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ambulance_1gazebo
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Mark executables and/or libraries for installation

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
