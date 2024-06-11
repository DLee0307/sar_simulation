cmake_minimum_required(VERSION 3.8)
project(sar_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(sar_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  include_CF_Firmware
  ${rclcpp_INCLUDE_DIRS}
  ${sar_msgs_INCLUDE_DIRS}
  ${sar_config_INCLUDE_DIRS}
)

# SAR_Controller
add_executable(SAR_Controller
    src/stabilizer.cpp
    src/Controller_GTC.cpp
    src/Shared_Lib.cpp
    src/Traj_Funcs.cpp)
ament_target_dependencies(SAR_Controller rclcpp std_msgs sar_msgs std_srvs yaml-cpp)

target_link_libraries(SAR_Controller yaml-cpp)

install(TARGETS
  SAR_Controller
  DESTINATION lib/${PROJECT_NAME}
)




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
