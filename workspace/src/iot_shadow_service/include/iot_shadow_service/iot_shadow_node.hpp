// Copyright ynyBonfennil. All Rights Reserved.
// SPDX-License-Identifier: MIT-0
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this
// software and associated documentation files (the "Software"), to deal in the Software
// without restriction, including without limitation the rights to use, copy, modify,
// merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
// PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
// HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
// OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
// SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#pragma once

#include <chrono>
#include <iostream>
#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>

#include <aws/crt/Api.h>
#include <aws/crt/UUID.h>
#include <aws/iot/MqttClient.h>
#include <aws/iotshadow/ErrorResponse.h>
#include <aws/iotshadow/IotShadowClient.h>
#include <aws/iotshadow/ShadowDeltaUpdatedEvent.h>
#include <aws/iotshadow/ShadowDeltaUpdatedSubscriptionRequest.h>
#include <aws/iotshadow/UpdateShadowRequest.h>
#include <aws/iotshadow/UpdateShadowResponse.h>
#include <aws/iotshadow/UpdateShadowSubscriptionRequest.h>

#include <aws/iotshadow/GetShadowRequest.h>
#include <aws/iotshadow/GetShadowResponse.h>
#include <aws/iotshadow/GetShadowSubscriptionRequest.h>
#include <aws/iotshadow/UpdateShadowSubscriptionRequest.h>

#include <iot_shadow_service_msgs/msg/shadow_update_snapshot.hpp>
#include <iot_shadow_service_msgs/srv/update_shadow.hpp>

class IoTShadowNode : public rclcpp::Node
{
private:
  // Initialize API
  Aws::Crt::ApiHandle api_handle_;

  std::string path_for_config_;
  bool discover_endpoints_;
  std::shared_ptr<Aws::Crt::Mqtt::MqttConnection> mqtt_connection_;

  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr spin_timer_;

  rclcpp::Service<iot_shadow_service_msgs::srv::UpdateShadow>::SharedPtr
    publish_to_shadow_srv_;
  rclcpp::Publisher<iot_shadow_service_msgs::msg::ShadowUpdateSnapshot>::SharedPtr
    shadow_update_snapshot_pub_;

  std::string thing_name_, shadow_name_;

private:
  void initMqttConnection();
  void connectToEndpoint(const nlohmann::json &);
  void connectUsingDiscovery(const nlohmann::json &);
  void onConnectionCompleted(Aws::Crt::Mqtt::MqttConnection &, int, Aws::Crt::Mqtt::ReturnCode, bool);
  void onInterrupted(Aws::Crt::Mqtt::MqttConnection &, int);
  void onResumed(Aws::Crt::Mqtt::MqttConnection &, Aws::Crt::Mqtt::ReturnCode, bool);
  void onDisconnect(Aws::Crt::Mqtt::MqttConnection &);

  void onDeltaUpdatedSubAck(int);
  void onDeltaUpdatedAcceptedSubAck(int);
  void onDeltaUpdatedRejectedSubAck(int);
  void onDeltaUpdated(Aws::Iotshadow::ShadowDeltaUpdatedEvent *, int);
  void onUpdateShadowAccepted(Aws::Iotshadow::UpdateShadowResponse *, int);
  void onUpdateShadowRejected(Aws::Iotshadow::ErrorResponse *, int);

  void onGetShadowUpdatedAcceptedSubAck(int);
  void onGetShadowUpdatedRejectedSubAck(int);
  void onGetShadowRequestSubAck(int);
  void onGetShadowAccepted(Aws::Iotshadow::GetShadowResponse *, int);
  void onGetShadowRejected(Aws::Iotshadow::ErrorResponse *, int);

  void initNodeInterfaces();
  void onShadowUpdateSnapshot(const iot_shadow_service_msgs::msg::ShadowUpdateSnapshot &);
  void onPublishToShadow(
    const std::shared_ptr<iot_shadow_service_msgs::srv::UpdateShadow::Request>,
    std::shared_ptr<iot_shadow_service_msgs::srv::UpdateShadow::Response>);

public:
  IoTShadowNode();
  ~IoTShadowNode();
};
