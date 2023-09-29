cmake_minimum_required(VERSION 3.8)
project(telemetry_mqtt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(
  mock_telemetry_publisher src/mock_telemetry_publisher.cpp)
ament_auto_add_executable(
  mqtt_telemetry_publisher src/mqtt_telemetry_publisher.cpp)

install(TARGETS
  mock_telemetry_publisher
  mqtt_telemetry_publisher)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
