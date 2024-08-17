cmake_minimum_required(VERSION 3.5)
project(laser_merger2)

set(PCL_INCLUDE_DIRS /usr/include/pcl-1.10)  

set(CUDA_MIN_VERSION "10.0")

add_definitions(-DUSE_LASER_NUM=2)

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
find_package(tf2_ros REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)
find_package(CUDA REQUIRED)

set(CUDA_SEPARABLE_COMPILATION ON)
set(CUDA_PROPAGATE_HOST_FLAGS OFF)
set(CUDA_ARCHITECTURES "native")

include_directories(
  ${CUDA_INCLUDE_DIRS} 
  include
)

link_directories(/usr/local/cuda/lib64)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# List of source files

cuda_add_executable(laser_merger2
  src/laser_merger2.cpp
  src/laser_merger2_main.cpp
  src/cuda_pointcloud.cu
)

ament_target_dependencies(laser_merger2 rclcpp tf2_ros tf2_sensor_msgs PCL pcl_conversions laser_geometry sensor_msgs)
target_link_libraries(laser_merger2 ${CUDA_LIBRARIES} cublas)

install(TARGETS
  laser_merger2
  DESTINATION lib/${PROJECT_NAME})

ament_package()

install(DIRECTORY include/
        DESTINATION include)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
