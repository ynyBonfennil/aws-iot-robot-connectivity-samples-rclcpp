# Note

## C++ SDK Initialization

In [mqtt_telemetry_publisher.hpp](https://github.com/ynyBonfennil/aws-iot-robot-connectivity-samples-rclcpp/blob/main/workspace/src/telemetry_mqtt/include/telemetry_mqtt/mqtt_telemetry_publisher.hpp), the following line plays a crucial role.

```cpp
Aws::Crt::ApiHandle api_handle_;
```

This line initializes the C++ SDK globally. If you skip this line, your SDK will not work properly.

## Relationship between the Simple MQTT Client and Device Shadow Node

Device shadow feature (named shadow feature) works ABOVE the MQTT connection layer, so both the simple MQTT client node and device shadow node do the same thing until they establish MQTT connection. The difference is AFTER the establishment of MQTT connection. Device shadow node subscribes topic related to device shadow feature to synchronize the state.

## Difference of Named Shadow Feature between C++ and Python

[iot_shadow_node implemented with Python](https://github.com/aws-samples/aws-iot-robot-connectivity-samples-ros2/blob/main/workspace/src/iot_shadow_service/iot_shadow_service/iot_shadow_node.py) uses `subscribe_to_named_shadow_updated_events` [at this line](https://github.com/aws-samples/aws-iot-robot-connectivity-samples-ros2/blob/0ef26465496d964d7ba408ceefb7253b5e9fa131/workspace/src/iot_shadow_service/iot_shadow_service/iot_shadow_node.py#L68), and this is a kind of high-level abstraction of the named device. It calls a callback function when names shadow is updated, and it provides `ShadowUpdatedEvent` which contains both desired value and reported value.

However, this abstraction layer is not implemented in C++ SDK. Hence we need to subscribe delta topic (`$aws/things/{thingName}/shadow/name/{shadowName}/update/delta`) and guess the desired value by comparing the reported value the device sends.

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
