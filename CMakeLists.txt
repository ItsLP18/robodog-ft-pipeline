cmake_minimum_required(VERSION 3.8)
project(ft_sensor_pipeline)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install Python nodes
install(
  DIRECTORY ft_sensor_pipeline/
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install scripts
install(
  PROGRAMS
    scripts/ft_sensor_driver_node.py
    scripts/ft_data_processor_node.py
    scripts/ft_data_logger_node.py
    scripts/ft_visualizer_node.py
    scripts/ft_mock_sensor_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
