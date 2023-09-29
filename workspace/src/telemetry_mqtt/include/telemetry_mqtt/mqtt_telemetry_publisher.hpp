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

#include <memory>
#include <string>
#include <fstream>

#include <aws/crt/Api.h>
#include <aws/crt/StlAllocator.h>
#include <aws/crt/auth/Credentials.h>
#include <aws/crt/io/TlsOptions.h>
#include <aws/iot/MqttClient.h>
#include <aws/crt/UUID.h>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>


class MqttTelemetryPublisher : public rclcpp::Node
{
private:
  std::string path_for_config_;
  bool discover_endpoints_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

private:
  void connectToEndpoint(const nlohmann::json &);
  void connectUsingDiscovery(const nlohmann::json &);
  void initSubs();
  void listenerCallback(const std_msgs::msg::String &);

public:
  MqttTelemetryPublisher();

};
