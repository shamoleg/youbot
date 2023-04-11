cmake_minimum_required(VERSION 3.0.2)
project(youbot_control)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
		controller_manager
		hardware_interface
		realtime_tools
		youbot_driver
		tf)

find_package(Boost REQUIRED COMPONENTS thread date_time filesystem system )

ADD_DEFINITIONS(-DYOUBOT_CONFIGURATIONS_DIR=${youbot_driver_CONFIG_DIR})

INCLUDE_DIRECTORIES(
	include
  	include/youbot_control
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR}
)

catkin_package(
  CATKIN_DEPENDS
  youbot_driver
  ros_robot_wrapper 
  roscpp
)

add_executable(${PROJECT_NAME}
				src/YouBotHW.cpp
				src/YouBotElementHW.cpp
				src/yuobot_hw_node.cpp)

add_dependencies(youbot_control ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
		${catkin_LIBRARIES}
		${Boost_LIBRARIES}
		)


install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
