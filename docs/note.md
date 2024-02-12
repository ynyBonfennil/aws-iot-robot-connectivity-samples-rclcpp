# Note

## Relationship between AWS Crt Cpp, AWS SDK for C++, and AWS IoT Device SDK for C++ v2

There are several libraries that can be used for AWS IoT;

- [AWS Crt Cpp](https://github.com/awslabs/aws-crt-cpp)
- [AWS SDK for C++](https://github.com/aws/aws-sdk-cpp)
- [AWS IoT Device SDK for C++ v2](https://github.com/aws/aws-iot-device-sdk-cpp-v2).

The relationships among them are

- AWS Crt Cpp is C++ wrapper of AWS Common Runtime (CRT) libraries, and is a basis of the others (both AWS SDK for C++ and AWS IoT Device SDK for C++ v2 are based on this)
- AWS SDK for C++ is the high level APIs for several AWS services
- AWS IoT Device SDK for C++ v2 is another high level APIs dedicated for AWS IoT
- [AWS Crt Cpp offers simple MQTT connection feature](https://github.com/awslabs/aws-crt-cpp/tree/main/include/aws/iot), but doesn't offer advanced features like device shadow, job etc.
- AWS SDK for C++ offers some features for AWS IoT, but basically it is almost the same as AWS Crt Cpp in terms of the APIs for AWS IoT
- AWS IoT Device SDK for C++ v2 offers useful tools for Device Shadow, Job, Secure Tunneling, Defender, Greengrass Discovery etc.

In short, you should use AWS IoT Device SDK for C++ v2 when using AWS IoT, and it is actually based on AWS Crt Cpp.

## Installation of AWS Crt Cpp

In this project, we use [aws_sdk_cpp_vendor](https://github.com/ros2-gbp/aws_sdk_cpp_vendor-release), which is a vendor package of AWS Crt Cpp for ROS2 environment. With this package, you only need to run `apt install ros-humble-aws-sdk-cpp-vendor` to install AWS Crt Cpp.

## Relationship between the Simple MQTT Client and Device Shadow Node

Device shadow feature (named shadow feature) works ABOVE the MQTT connection layer, so both the simple MQTT client node and device shadow node do the same thing until they establish MQTT connection. The difference is AFTER the establishment of MQTT connection. Device shadow node subscribes topic related to device shadow feature to synchronize the state.

## Difference of Named Shadow Feature between C++ and Python

[iot_shadow_node implemented with Python](https://github.com/aws-samples/aws-iot-robot-connectivity-samples-ros2/blob/main/workspace/src/iot_shadow_service/iot_shadow_service/iot_shadow_node.py) uses `subscribe_to_named_shadow_updated_events` [at this line](https://github.com/aws-samples/aws-iot-robot-connectivity-samples-ros2/blob/0ef26465496d964d7ba408ceefb7253b5e9fa131/workspace/src/iot_shadow_service/iot_shadow_service/iot_shadow_node.py#L68), and this is a kind of high-level abstraction of the named device. It calls a callback function when names shadow is updated, and it provides `ShadowUpdatedEvent` which contains both desired value and reported value.

However, this abstraction layer is not implemented in AWS IoT Device SDK for C++ v2. Hence we need to subscribe delta topic (`$aws/things/{thingName}/shadow/name/{shadowName}/update/delta`) and guess the desired value by comparing the reported value the device sends.

### Get shadow document

While delta topic is ideal to quickly notify the update of shadow document, sometimes you would want to get the whole shadow document instead of the diff information. This can be done by using Get Topics

- `$aws/things/{tingName}/shadow/name/{shadowName}/get`
  - To request the current state of the shadow
- `$aws/things/{tingName}/shadow/name/{shadowName}/get/accepted`
  - Response to a get request, containing the shadow document
- `$aws/things/{tingName}/shadow/name/{shadowName}/get/rejected`
  - Notification that a get request was rejected

This is just like a HTTP GET.

FYI, named shadow also offers topics that look like HTTP POST and HTTP DELETE. HTTP POST is often used when the device sends "reported" value to server.

## C++ SDK Initialization

In [mqtt_telemetry_publisher.hpp](https://github.com/ynyBonfennil/aws-iot-robot-connectivity-samples-rclcpp/blob/main/workspace/src/telemetry_mqtt/include/telemetry_mqtt/mqtt_telemetry_publisher.hpp), the following line plays a crucial role.

```cpp
Aws::Crt::ApiHandle api_handle_;
```

This line initializes the C++ SDK globally. If you skip this line, your SDK will not work properly.
