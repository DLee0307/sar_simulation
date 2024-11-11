cmake_minimum_required(VERSION 3.8)
project(sar_data_converter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sar_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Curses REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(crazyflie_interfaces REQUIRED)

find_package(gz-sim8 REQUIRED)
find_package(gz-sensors8 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${sar_msgs_INCLUDE_DIRS}
  ${sar_config_INCLUDE_DIRS}
  ${CURSES_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
 )

# SAR_DataConverter
add_executable(SAR_DataConverter 
	src/SAR_DataConverter.cpp
	src/Data_Parsing.cpp
	src/Data_Summary.cpp
	src/Debug_Console.cpp
	src/Logging_Funcs.cpp
	src/Gazebo_Funcs.cpp)
ament_target_dependencies(SAR_DataConverter 
	rclcpp 
	std_msgs 
	sar_msgs 
	std_srvs 
	crazyflie_interfaces
  	gz-sim8
  	gz-sensors8
	)


target_link_libraries(SAR_DataConverter ${CURSES_LIBRARIES} Eigen3::Eigen)

install(TARGETS
  SAR_DataConverter
  DESTINATION lib/${PROJECT_NAME}
)
  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
