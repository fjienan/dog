cmake_minimum_required(VERSION 3.8)
project(ares_comm)

# ─────────────────── 编译警告开关 ───────────────────
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ─────────────────── 依赖包 ───────────────────
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime   REQUIRED)
find_package(rclcpp                   REQUIRED)
find_package(geometry_msgs            REQUIRED)
find_package(nav_msgs                 REQUIRED)
find_package(tf2                      REQUIRED)        # ← 新增，控制节点订阅 Odometry

# ─────────────────── 生成接口 ───────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Data.msg"
  "msg/Error.msg"
  "srv/Execute.srv"
)

# ─────────────────── 头文件目录 ───────────────────
include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

# ─────────────────── 可执行文件 ───────────────────
## ① 串口通信节点
add_executable(ares_comm_node
  src/ares_comm_node.cpp
  src/crc.cpp
  src/protocol_parser.cpp
  src/serial_interface.cpp
)
## ② IMU 传感器节点
add_executable(imu_sensor_node
  src/imu_sensor_node.cpp
  src/crc.cpp
  src/protocol_parser.cpp
  src/serial_interface.cpp
)
## ③ ★ 新增：速度控制节点
add_executable(ares_control_node
  src/ares_control_node.cpp
)

# ─────────────────── 关联接口库 ───────────────────
rosidl_target_interfaces(ares_comm_node     ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(imu_sensor_node    ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(ares_control_node  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ─────────────────── 依赖设置 ───────────────────
ament_target_dependencies(ares_comm_node
  rclcpp rosidl_default_runtime geometry_msgs nav_msgs tf2
)
ament_target_dependencies(imu_sensor_node
  rclcpp rosidl_default_runtime geometry_msgs nav_msgs tf2
)
ament_target_dependencies(ares_control_node
  rclcpp rosidl_default_runtime geometry_msgs nav_msgs tf2
)

# ─────────────────── C++14 打开 ───────────────────
target_compile_features(ares_comm_node    PUBLIC cxx_std_14)
target_compile_features(imu_sensor_node   PUBLIC cxx_std_14)
target_compile_features(ares_control_node PUBLIC cxx_std_14)

# ─────────────────── 安装 ───────────────────
install(TARGETS
  ares_comm_node
  imu_sensor_node
  ares_control_node           # ← 新增
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  include/
  DESTINATION include
)

# ─────────────────── 导出 ───────────────────
ament_export_dependencies(
  rosidl_default_runtime
  rclcpp
  geometry_msgs
  nav_msgs
)
ament_export_include_directories(include)

# ─────────────────── 结束 ───────────────────
ament_package()