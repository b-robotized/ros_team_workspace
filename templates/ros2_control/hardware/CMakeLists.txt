cmake_minimum_required(VERSION 3.16)
project(dummy_package_namespace)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=return-type)
endif()

set(THIS_PACKAGE_DEPS
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
foreach(PACKAGE ${THIS_PACKAGE_DEPS})
  find_package(${PACKAGE} REQUIRED)
endforeach()

add_library(dummy_file_name SHARED
  src/dummy_file_name.cpp
)

target_include_directories(dummy_file_name
  PUBLIC
    include
)

target_link_libraries(dummy_file_name
  PUBLIC
    hardware_interface::hardware_interface
    pluginlib::pluginlib
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
)

target_compile_definitions(dummy_file_name PUBLIC
  "PLUGINLIB__DISABLE_BOOST_FUNCTIONS"
)

pluginlib_export_plugin_description_file(hardware_interface
  dummy_package_namespace.xml
)

install(
  TARGETS dummy_file_name
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)
  ament_add_gmock(test_dummy_file_name test/test_dummy_file_name.cpp)
  target_link_libraries(test_dummy_file_name dummy_file_name
    rclcpp::rclcpp
    ros2_control_test_assets::ros2_control_test_assets
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(dummy_file_name)
ament_export_dependencies(${THIS_PACKAGE_DEPS})

ament_package()
