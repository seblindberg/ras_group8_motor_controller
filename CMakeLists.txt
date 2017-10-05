cmake_minimum_required(VERSION 2.8.3)
project(ras_group8_motor_controller)

set(RAS_GROUP8_MOTOR_CONTROLLER_PUBLISH_PID 1)

find_package(catkin REQUIRED
	COMPONENTS
	roscpp
	geometry_msgs
	std_msgs
	std_srvs
)

catkin_package(
	INCLUDE_DIRS
		include
	
	CATKIN_DEPENDS
		roscpp
		geometry_msgs
		std_msgs
		std_srvs
	
# DEPENDS
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
	src/motor_controller_node.cpp
	src/PIDController.cpp
	src/MotorController.cpp
)

add_dependencies(${PROJECT_NAME}
	${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
)

#catkin_add_gtest(motor_controller_tests test/test_motor_controller.cpp src/MotorController.cpp)

# Make main executable
install(
  TARGETS ${PROJECT_NAME}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install headers
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)