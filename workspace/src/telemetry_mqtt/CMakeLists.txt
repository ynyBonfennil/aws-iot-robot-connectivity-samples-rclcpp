cmake_minimum_required(VERSION 3.8)
project(telemetry_mqtt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# AWS IoT SDK
set(AWS_CMAKE_DIRS
  $ENV{HOME}/sdk-workspace/lib/EventstreamRpc-cpp/cmake
  $ENV{HOME}/sdk-workspace/lib/IotIdentity-cpp/cmake
  $ENV{HOME}/sdk-workspace/lib/aws-c-mqtt/cmake
  $ENV{HOME}/sdk-workspace/lib/aws-c-http/cmake
  $ENV{HOME}/sdk-workspace/lib/aws-c-io/cmake
  $ENV{HOME}/sdk-workspace/lib/aws-crt-cpp/cmake
  $ENV{HOME}/sdk-workspace/lib/aws-c-auth/cmake
  $ENV{HOME}/sdk-workspace/lib/aws-c-sdkutils/cmake
  $ENV{HOME}/sdk-workspace/lib/aws-checksums/cmake
  $ENV{HOME}/sdk-workspace/lib/aws-c-iot/cmake
  $ENV{HOME}/sdk-workspace/lib/aws-c-compression/cmake
  $ENV{HOME}/sdk-workspace/lib/IotDeviceCommon-cpp/cmake
  $ENV{HOME}/sdk-workspace/lib/GreengrassIpc-cpp/cmake
  $ENV{HOME}/sdk-workspace/lib/s2n/cmake
  $ENV{HOME}/sdk-workspace/lib/aws-c-s3/cmake
  $ENV{HOME}/sdk-workspace/lib/aws-c-event-stream/cmake
  $ENV{HOME}/sdk-workspace/lib/aws-c-common/cmake
  $ENV{HOME}/sdk-workspace/lib/IotJobs-cpp/cmake
  $ENV{HOME}/sdk-workspace/lib/aws-c-cal/cmake
  $ENV{HOME}/sdk-workspace/lib/IotShadow-cpp/cmake
  $ENV{HOME}/sdk-workspace/lib/IotSecureTunneling-cpp/cmake
  $ENV{HOME}/sdk-workspace/lib/Discovery-cpp/cmake
  $ENV{HOME}/sdk-workspace/lib/IotDeviceDefender-cpp/cmake
)
foreach(DIR ${AWS_CMAKE_DIRS})
  list(APPEND CMAKE_PREFIX_PATH ${DIR})
endforeach()
find_package(aws-crt-cpp REQUIRED)

set(AWS_SDK_INCLUDE_DIR $ENV{HOME}/sdk-workspace/include)
set(AWS_SDK_LIB_DIR $ENV{HOME}/sdk-workspace/lib)
include_directories(${AWS_SDK_INCLUDE_DIR})
link_directories(${AWS_SDK_LIB_DIR})

ament_auto_add_executable(
  mock_telemetry_publisher src/mock_telemetry_publisher.cpp)
ament_auto_add_executable(
  mqtt_telemetry_publisher src/mqtt_telemetry_publisher.cpp)
target_link_libraries(
  mqtt_telemetry_publisher AWS::aws-crt-cpp)

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
