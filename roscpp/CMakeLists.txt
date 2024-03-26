cmake_minimum_required(VERSION 3.8)
project(roscpp)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros2_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

# include_directories(/usr/local/include)
add_executable(pub_pos src/pub_pos.cpp)
target_link_libraries(pub_pos libRVO.a)
ament_target_dependencies(pub_pos rclcpp std_msgs ros2_interfaces geometry_msgs)

install(TARGETS
  pub_pos
  DESTINATION lib/${PROJECT_NAME})

# python part  
ament_python_install_package(${PROJECT_NAME})
  
install(PROGRAMS
  scripts/draw_plot.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
