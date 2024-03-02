cmake_minimum_required(VERSION 3.8)
project(ros2_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/Ctrl.msg"
    "msg/Motions.msg"
    "msg/State.msg"
    "msg/Yscomm.msg"
    "msg/Mocap.msg"
    "msg/YsCtrl.msg"
    "srv/AgentStatus.srv"
    "srv/Comm.srv"
    DEPENDENCIES geometry_msgs
  )
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
