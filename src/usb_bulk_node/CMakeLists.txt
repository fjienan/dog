cmake_minimum_required(VERSION 3.10)
project(usb_bulk_node LANGUAGES CXX)
# 在 project() 之后立即添加
message(STATUS "===== 开始配置 =====")  # 测试基础输出功能

# 设置 C++ 标准
# set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_CXX_STANDARD_REQUIRED True)

# 查找所有依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)

# # 在find_package之后添加路径输出
# message( "==============================================")
# message( "rclcpp 包路径信息:")
# message( "  - 配置文件路径: ${rclcpp_DIR}")
# message( "  - 包含目录: ${rclcpp_INCLUDE_DIRS}")
# # message( "  - 库文件: ${rclcpp_LIBRARIES}")

# message( "std_msgs 包路径信息:")
# message( "  - 配置文件路径: ${std_msgs_DIR}")
# message( "  - 包含目录: ${std_msgs_INCLUDE_DIRS}")
# # message( "  - 库文件: ${std_msgs_LIBRARIES}")
# message( "==============================================")

# ------------------ 链接验证逻辑 --------------------
# 检查rclcpp是否找到
# if(NOT rclcpp_FOUND)
#   message(FATAL_ERROR "[ERROR] rclcpp 未找到，请检查ROS2环境")
# else()
#   message(STATUS "[SUCCESS] rclcpp 已找到: ${rclcpp_DIR}")
# endif()

# # 检查std_msgs是否找到
# if(NOT std_msgs_FOUND)
#   message(FATAL_ERROR "[ERROR] std_msgs 未找到，请安装ros-$ROS_DISTRO-std-msgs")
# else()
#   message(STATUS "[SUCCESS] std_msgs 已找到: ${std_msgs_DIR}")
# endif()

# 验证目标是否存在（关键检查）
# if(NOT TARGET rclcpp::rclcpp)
#   message(FATAL_ERROR "[ERROR] rclcpp目标未正确导出，尝试重新安装rclcpp")
# endif()

# if(NOT TARGET std_msgs::std_msgs)
#   message(FATAL_ERROR "[ERROR] std_msgs目标未正确导出，尝试重新安装std_msgs")
# endif()
# ---------------------------------------------------

# 配置libusb
pkg_check_modules(LIBUSB_1 REQUIRED libusb-1.0)

# 创建可执行文件
add_executable(usb_bulk_node src/usb_bulk_node.cpp src/ares_protocol.cpp)
ament_target_dependencies(usb_bulk_node 
  rclcpp 
  geometry_msgs
  )

# 统一链接配置,包含src/ares_protocol.hpp,我转移到了include/usb_bulk_node目录下面
target_include_directories(usb_bulk_node 
  PUBLIC
  ${LIBUSB_1_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/usb_bulk_node
  ${geometry_msgs_INCLUDE_DIRS}

)

target_link_libraries(usb_bulk_node 
  # 系统库
  usb-1.0
  ${LIBUSB_1_LIBRARIES}
  Threads::Threads
)



# 测试配置
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
# 安装配置
install(TARGETS usb_bulk_node
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()