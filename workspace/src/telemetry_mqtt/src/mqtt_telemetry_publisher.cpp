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

#include <telemetry_mqtt/mqtt_telemetry_publisher.hpp>

MqttTelemetryPublisher::MqttTelemetryPublisher()
: rclcpp::Node("mqtt_telemetry_publisher")
{
  this->declare_parameter<std::string>("path_for_config", "");
  this->declare_parameter<bool>("discover_endpoints", false);

  this->get_parameter("path_for_config", this->path_for_config_);
  this->get_parameter("discover_endpoints", this->discover_endpoints_);

  std::ifstream config_file(this->path_for_config_);
  nlohmann::json cert_data;
  config_file >> cert_data;
  RCLCPP_INFO(this->get_logger(), "Config we are loading is :\n%s", cert_data.dump().c_str());

  if (this->discover_endpoints_) {
    RCLCPP_INFO(this->get_logger(), "Discovering endpoints for connection");
    this->connectUsingDiscovery(cert_data);
  } else {
    RCLCPP_INFO(this->get_logger(), "Connecting directly to endpoint");
    this->connectToEndpoint(cert_data);
  }

  this->initSubs();
}

void MqttTelemetryPublisher::connectToEndpoint(const nlohmann::json & cert_data)
{

}

void MqttTelemetryPublisher::connectUsingDiscovery(const nlohmann::json & cert_data)
{

}

void MqttTelemetryPublisher::initSubs()
{
  this->subscription_ = this->create_subscription<std_msgs::msg::String>(
    "mock_telemetry",
    10,
    std::bind(&MqttTelemetryPublisher::listenerCallback, this, std::placeholders::_1));
}

void MqttTelemetryPublisher::listenerCallback(const std_msgs::msg::String & msg)
{

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MqttTelemetryPublisher>());
  rclcpp::shutdown();
  return 0;
}
