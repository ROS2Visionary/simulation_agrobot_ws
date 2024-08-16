cmake_minimum_required(VERSION 3.8)
project(agrobot_arm_controller)

# 如果使用GNU编译器或Clang编译器，则添加编译选项以显示所有警告信息
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 找到所需的依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# 包含目录
include_directories(include)

# 添加可执行文件
add_executable(demo src/demo.cpp)

# 链接库
ament_target_dependencies(demo
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  moveit_core
  moveit_ros_planning_interface
)

# 安装可执行文件
install(TARGETS demo
  DESTINATION lib/${PROJECT_NAME})

# 测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 跳过版权检查
  set(ament_cmake_copyright_FOUND TRUE)
  # 跳过cpplint检查（仅适用于git仓库）
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
