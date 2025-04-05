cmake_minimum_required(VERSION 3.8)
project(ares_comm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# 生成消息和服务接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Data.msg"
  "msg/Error.msg"
  "srv/Execute.srv"
)

# 包含目录
include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

# 添加可执行文件
add_executable(ares_comm_node
  src/ares_comm_node.cpp
  src/crc.cpp
  src/protocol_parser.cpp  
  src/serial_interface.cpp
)

add_executable(imu_sensor_node
  src/imu_sensor_node.cpp
  src/crc.cpp
  src/protocol_parser.cpp  
  src/serial_interface.cpp
)

# 链接消息接口库
rosidl_target_interfaces(ares_comm_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

rosidl_target_interfaces(imu_sensor_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# 设置依赖
ament_target_dependencies(ares_comm_node
  rclcpp
  rosidl_default_runtime
  geometry_msgs
)

ament_target_dependencies(imu_sensor_node
  rclcpp
  rosidl_default_runtime
  geometry_msgs
)

# 设置C++标准
target_compile_features(ares_comm_node PUBLIC cxx_std_14)
target_compile_features(imu_sensor_node PUBLIC cxx_std_14)

# 安装可执行文件
install(TARGETS
  ares_comm_node
  imu_sensor_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include
)

# 导出依赖
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(rclcpp)
ament_export_dependencies(geometry_msgs)
ament_export_include_directories(include)

# 构建包
ament_package()
