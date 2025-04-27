cmake_minimum_required(VERSION 3.8)
project(custom_geometry_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(ROSIDL_GENERATE_PYTHON TRUE)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
# 生成消息接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PlanarPose.msg"
  DEPENDENCIES geometry_msgs builtin_interfaces  # 声明依赖 geometry_msgs
)

# 添加可执行文件
add_executable(planar_pose_publisher src/planar_pose_publisher.cpp)

ament_target_dependencies(planar_pose_publisher 
  "rclcpp"  
  "geometry_msgs"
)

# 新增：将生成的消息接口链接到可执行文件
rosidl_target_interfaces(planar_pose_publisher
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 安装可执行文件
install(TARGETS planar_pose_publisher
  DESTINATION lib/${PROJECT_NAME}
)

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

