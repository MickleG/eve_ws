cmake_minimum_required(VERSION 3.0.2)
project(eve_main)

find_package(catkin REQUIRED COMPONENTS
  dynamixel_sdk
  roscpp
  std_msgs
	message_generation
)

add_message_files(
	FILES
	EndEffectorPosition.msg
)

add_service_files(
	FILES
	GetPosition.srv
	GoToPosition.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED PATHS /usr/local/lib/cmake/realsense2/)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES eve_main
  CATKIN_DEPENDS dynamixel_sdk roscpp std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
	${realsense2_INCLUDE_DIRS}
  ~/DynamixelSDK/c++/include/dynamixel_sdk
)

add_executable(eve_main_node
  src/eve_main_node.cpp
  src/MotorConfig.cpp
  src/EndEffectorConfig.cpp
  src/Grip.cpp
  src/Cutter.cpp
)

add_executable(image_processing_node
	src/image_processing_node.cpp
)

add_executable(motor_node
	src/motor_node.cpp
	src/MotorConfig.cpp
	src/EndEffectorConfig.cpp
)

add_executable(test_y
	src/test_y.cpp
)

target_link_libraries(eve_main_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
	realsense2::realsense2
  -lwiringPi
)

target_link_libraries(image_processing_node
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	realsense2::realsense2
	-lwiringPi
)

target_link_libraries(motor_node
	${catkin_LIBRARIES}
	-lwiringPi
)

target_link_libraries(test_y
	${catkin_LIBRARIES}
)

add_dependencies(eve_main_node
	${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)

add_dependencies(motor_node
	${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)

add_dependencies(test_y
	${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)
