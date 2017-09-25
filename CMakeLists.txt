cmake_minimum_required(VERSION 2.8.3)
project(ras_group8_motor_controller)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs)

catkin_package(
	DEPENDS
	CATKIN_DEPENDS roscpp geometry_msgs
	INCLUDE_DIRS include
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_executable(motor_controller src/motor_controller_node.cpp)
add_dependencies(motor_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(motor_controller ${catkin_LIBRARIES})

#catkin_add_gtest(motor_controller_tests test/motor_controller_test.cpp src/motor_controller.cpp)

install(TARGETS motor_controller
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
