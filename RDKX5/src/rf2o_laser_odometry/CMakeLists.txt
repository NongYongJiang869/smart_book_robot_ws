cmake_minimum_required(VERSION 3.5)
project(rf2o_laser_odometry)

# Default to C++17 (ROS 2 Humble standard)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── ROS 2 dependencies ──
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# ── System deps (headers only, no linking needed) ──
find_package(Boost REQUIRED)

# ── Executable ──
set(EXECUTABLE_NAME "rf2o_laser_odometry_node")

add_executable(${EXECUTABLE_NAME}
    src/CLaserOdometry2DNode.cpp
    src/CLaserOdometry2D.cpp)

target_include_directories(${EXECUTABLE_NAME} PRIVATE
    include/
    ${Boost_INCLUDE_DIRS})

set(DEPENDENCIES
    "rclcpp"
    "std_msgs"
    "sensor_msgs"
    "nav_msgs"
    "geometry_msgs"
    "tf2"
    "tf2_ros"
    "tf2_geometry_msgs"
    "Eigen3"
)

ament_target_dependencies(${EXECUTABLE_NAME} ${DEPENDENCIES})

# ── Install ──
install(TARGETS ${EXECUTABLE_NAME}
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME})

ament_package()
