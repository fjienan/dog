cmake_minimum_required(VERSION 3.8)
project(my_robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install( # 这个是你自己加的
  DIRECTORY worlds models urdf launch rviz meshes config  # 要安装的目录们
  DESTINATION share/${PROJECT_NAME}/ # 要安装到的位置，将会放到 install/my_robot_description/share/my_robot_description 里
# ${PROJECT_NAME} 即上面写的 NAME
)

ament_package()
