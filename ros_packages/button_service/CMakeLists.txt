cmake_minimum_required(VERSION 3.8)
project(button_service)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetButtonColor.srv"
  "srv/ReadButton.srv"
  "srv/WaitForButton.srv"
)

ament_python_install_package(button_service_node_pkg)

install(PROGRAMS
  scripts/button_service_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
