cmake_minimum_required(VERSION 3.8)
project(ls01g)

# 查找必要的包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 默认构建类型
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(
  include
)

# 添加 ls01g_driver 共享库
add_library(ls01g_driver SHARED
  src/ls01g_driver.cpp
)

# 为 ls01g_driver 库设置属性
set_target_properties(ls01g_driver PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  VERSION 0.1.0
  SOVERSION 0.1
)

# 添加可执行文件，只包含 ls01g_node.cpp
add_executable(ls01g_node 
  src/ls01g_node.cpp
)

# 链接依赖
target_link_libraries(ls01g_node 
  ls01g_driver
)

# 设置 ROS2 依赖
ament_target_dependencies(ls01g_node
  rclcpp
  std_msgs
  sensor_msgs
)

# 库可能也需要一些依赖项
ament_target_dependencies(ls01g_driver
  std_msgs
)

# 安装目标 - 可执行文件和库
install(TARGETS 
  ls01g_node 
  ls01g_driver
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# 安装启动文件和其他资源
install(DIRECTORY 
  launch 
  xacro
  bag
  # urdf 
  meshes 
  rviz
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY include
  DESTINATION include
)

# 导出包信息
ament_export_libraries(ls01g_driver)
ament_export_dependencies(rclcpp std_msgs sensor_msgs)
ament_package()