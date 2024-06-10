cmake_minimum_required(VERSION 3.14)
project(obstacle_detection)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  include/obstacle_detection/obstacle_detection.hpp
  include/obstacle_detection/visibility_control.hpp
  src/obstacle_detection.cpp
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  include/obstacle_detection/obstacle_detection_node.hpp
  src/obstacle_detection_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "obstacle_detection::ObstacleDetectionNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_ros_isolated_gtest(test_obstacle_detection
    test/test_obstacle_detection.cpp
  )
  ament_target_dependencies(test_obstacle_detection ${TEST_SOURCES})
  target_link_libraries(test_obstacle_detection ${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
