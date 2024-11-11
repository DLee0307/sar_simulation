cmake_minimum_required(VERSION 3.5)
project(sar_gazebo)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-sensors8 REQUIRED)
find_package(sar_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gz-msgs10 REQUIRED)

gz_find_package(gz-physics7
  COMPONENTS
    heightmap
    mesh
    sdf
  REQUIRED
)
set(GZ_PHYSICS_VER ${gz-physics7_VERSION_MAJOR})

gz_find_package(gz-math7 REQUIRED COMPONENTS eigen3)
set(GZ_MATH_VER ${gz-math7_VERSION_MAJOR})

# Find gz-plugin
gz_find_package(gz-plugin2 REQUIRED COMPONENTS loader register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

include_directories(
  plugins/include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sar_msgs_INCLUDE_DIRS}
  ${gz-sensors8_INCLUDE_DIRS}
  ${gz-msgs10_INCLUDE_DIRS}
)

# Practice
add_library(Practice SHARED plugins/src/Practice.cpp)

ament_target_dependencies(Practice
  gz-sim8
  gz-sensors8
  rclcpp
)

target_link_libraries(Practice
  gz-sim8::gz-sim8
  gz-sensors8::gz-sensors8
  gz-msgs10::gz-msgs10
  ${rclcpp_LIBRARIES}

)

install(TARGETS Practice
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

# Step_World_Plugin
add_library(Step_World_Plugin SHARED plugins/src/Step_World_Plugin.cpp)

ament_target_dependencies(Step_World_Plugin
  gz-sim8
  gz-sensors8
  rclcpp
  sar_msgs
)

target_link_libraries(Step_World_Plugin
  gz-sim8::gz-sim8
  gz-sensors8::gz-sensors8
  gz-msgs10::gz-msgs10
  ${rclcpp_LIBRARIES}

)

install(TARGETS Step_World_Plugin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

# Camera_Plugin
add_library(Camera_Plugin SHARED plugins/src/Camera_Plugin.cpp)

ament_target_dependencies(Camera_Plugin
  gz-sim8
  gz-sensors8
  rclcpp
  sar_msgs
)

target_link_libraries(Camera_Plugin
  gz-sim8::gz-sim8
  gz-sensors8::gz-sensors8
  gz-msgs10::gz-msgs10
  ${rclcpp_LIBRARIES}

)

install(TARGETS Camera_Plugin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

# Sticky_Leg_Plugin
add_library(Sticky_Leg_Plugin SHARED plugins/src/Sticky_Leg_Plugin.cpp)

ament_target_dependencies(Sticky_Leg_Plugin
  gz-sim8
  gz-sensors8
  rclcpp
  std_msgs
  sar_msgs
  geometry_msgs
)

target_link_libraries(Sticky_Leg_Plugin
  gz-sim8::gz-sim8
  gz-sensors8::gz-sensors8
  ${rclcpp_LIBRARIES}

)

install(TARGETS Sticky_Leg_Plugin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

# Motor Plugin
add_library(Motor_Plugin SHARED plugins/src/Motor_Plugin.cpp)

ament_target_dependencies(Motor_Plugin
  gz-sim8
  gz-sensors8
  rclcpp
  std_msgs
  sar_msgs
)

target_link_libraries(Motor_Plugin
  gz-sim8::gz-sim8
  gz-sensors8::gz-sensors8
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
)

install(TARGETS Motor_Plugin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

# Imu_Plugin
add_library(Imu_Plugin SHARED plugins/src/Imu_Plugin.cpp)

ament_target_dependencies(Imu_Plugin
  gz-sim8
  gz-sensors8
  rclcpp
  std_msgs
  sar_msgs
  geometry_msgs
)

target_link_libraries(Imu_Plugin
  gz-sim8::gz-sim8
  gz-sensors8::gz-sensors8
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
)

install(TARGETS Imu_Plugin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

# Vicon_Plugin
add_library(Vicon_Plugin SHARED plugins/src/Vicon_Plugin.cpp)

ament_target_dependencies(Vicon_Plugin
  gz-sim8
  gz-sensors8
  rclcpp
  std_msgs
  sar_msgs
  geometry_msgs
)

target_link_libraries(Vicon_Plugin
  gz-sim8::gz-sim8
  gz-sensors8::gz-sensors8
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
)

install(TARGETS Vicon_Plugin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
