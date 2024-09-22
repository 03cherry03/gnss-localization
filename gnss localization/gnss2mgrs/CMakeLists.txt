cmake_minimum_required(VERSION 3.5)
project(gnss2mgrs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geodesy REQUIRED)  # geodesy 패키지 추가
find_package(PkgConfig REQUIRED)

# Check for proj4 package
pkg_check_modules(PROJ4 REQUIRED proj)

# Add include directories (including custom include directory for mgrs.h)
include_directories(include ${PROJ4_INCLUDE_DIRS})

# Alternatively, use target_include_directories for better target-specific inclusion
# target_include_directories(gnss2mgrs_node PUBLIC ${PROJECT_SOURCE_DIR}/include ${PROJ4_INCLUDE_DIRS})

# Build the executable
add_executable(gnss2mgrs_node src/gnss2mgrs.cpp)

# Link dependencies
ament_target_dependencies(gnss2mgrs_node 
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
  geographic_msgs
  geodesy 
)

# Link proj4 library
target_link_libraries(gnss2mgrs_node ${PROJ4_LIBRARIES})

# Install targets and directories
install(TARGETS
  gnss2mgrs_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
