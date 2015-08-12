cmake_minimum_required(VERSION 2.8.3)
project(ros_synced_image_provider)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  cv_bridge
)

find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(OpenCV)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_synced_image_provider
#  CATKIN_DEPENDS image_transport roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(synced_image_provider_node src/image_provider.cpp)
add_dependencies(synced_image_provider_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(synced_image_provider_node 
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
)

install(TARGETS synced_image_provider_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)