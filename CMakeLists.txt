cmake_minimum_required(VERSION 3.8)
project(rm_robot_recognizer)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs   REQUIRED)                         # 点云消息类型
find_package(geometry_msgs REQUIRED)                         # 发布 Pose2D  
find_package(visualization_msgs REQUIRED)                    # 可视化 MarkerArray  
find_package(pcl_conversions REQUIRED)                       # PCL 与 ROS 点云互转 :contentReference[oaicite:2]{index=2}
find_package(pcl_ros          REQUIRED)                      # PCL-ROS 接口 :contentReference[oaicite:3]{index=3}
find_package(nav_msgs REQUIRED)
add_executable(robot_recognizer
              src/robot_recognizer.cpp)   

target_include_directories(robot_recognizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(robot_recognizer
    rclcpp
    sensor_msgs
    geometry_msgs
    visualization_msgs
    pcl_conversions
    pcl_ros
    nav_msgs
    )

  install(TARGETS robot_recognizer
          DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY
  	launch
  	config
  	DESTINATION share/${PROJECT_NAME}/)
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
