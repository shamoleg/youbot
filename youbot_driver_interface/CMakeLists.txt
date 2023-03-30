cmake_minimum_required(VERSION 3.0.2)
project(youbot_driver_interface)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
		ros_robot_wrapper
		youbot_driver
		controller_manager
		tf)

find_package(Boost REQUIRED COMPONENTS thread date_time filesystem system )

ADD_DEFINITIONS(-DYOUBOT_CONFIGURATIONS_DIR=${youbot_driver_CONFIG_DIR})

INCLUDE_DIRECTORIES(
	include
  	include/youbot_driver_interface
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR}
)

catkin_package(
  CATKIN_DEPENDS  
  youbot_driver
  ros_robot_wrapper 
  roscpp
)

add_executable(youbot_driver_interface
				src/YouBotHW.cpp
				src/yuobot_hw_node.cpp)

add_dependencies(youbot_driver_interface ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
		${catkin_LIBRARIES}
		${Boost_LIBRARIES}
		)

#add_executable(yuobot_hw_node
#		src/YouBotHW.cpp
#		src/yuobot_hw_node.cpp)
#add_dependencies(yuobot_hw_node ${catkin_EXPORTED_TARGETS})
#
#target_link_libraries(yuobot_hw_node
#		${catkin_LIBRARIES}
#		${Boost_LIBRARIES}
#		)


install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
