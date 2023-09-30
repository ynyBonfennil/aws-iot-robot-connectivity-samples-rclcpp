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

  using namespace std::chrono_literals; // NOLINT
  this->init_timer_ = this->create_wall_timer(
    0s, std::bind(&MqttTelemetryPublisher::initMqttConnection, this));
}

MqttTelemetryPublisher::~MqttTelemetryPublisher()
{
  this->subscription_.reset();

  this->mqtt_connection_->Disconnect();
}

void MqttTelemetryPublisher::initMqttConnection()
{
  this->init_timer_.reset();

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
}

void MqttTelemetryPublisher::connectToEndpoint(const nlohmann::json & cert_data)
{
  auto client_config_builder = Aws::Iot::MqttClientConnectionConfigBuilder(
    cert_data["certificatePath"].get<std::string>().c_str(),
    cert_data["privateKeyPath"].get<std::string>().c_str());
  client_config_builder.WithEndpoint(cert_data["endpoint"].get<std::string>().c_str());
  client_config_builder.WithCertificateAuthority(cert_data["rootCAPath"].get<std::string>().c_str());
  client_config_builder.WithPortOverride(cert_data["port"].get<uint16_t>());

  Aws::Iot::MqttClientConnectionConfig client_config = client_config_builder.Build();
  if (!client_config) {
    RCLCPP_ERROR(
      this->get_logger(), "MQTT client configuration initialization failed with error %s",
      Aws::Crt::ErrorDebugString(client_config.LastError()));
    std::exit(-1);
  }

  Aws::Iot::MqttClient mqtt_client = Aws::Iot::MqttClient();
  this->mqtt_connection_ = mqtt_client.NewConnection(client_config);
  if (!*this->mqtt_connection_) {
    RCLCPP_ERROR(
      this->get_logger(), "MQTT connection creation failed with error %s",
      Aws::Crt::ErrorDebugString(this->mqtt_connection_->LastError()));
    std::exit(-1);
  }

  using namespace std::placeholders;
  this->mqtt_connection_->OnConnectionCompleted = std::bind(
    &MqttTelemetryPublisher::onConnectionCompleted, this, _1, _2, _3, _4);
  this->mqtt_connection_->OnDisconnect = std::bind(
    &MqttTelemetryPublisher::onDisconnect, this, _1);
  this->mqtt_connection_->OnConnectionInterrupted = std::bind(
    &MqttTelemetryPublisher::onInterrupted, this, _1, _2);
  this->mqtt_connection_->OnConnectionResumed = std::bind(
    &MqttTelemetryPublisher::onResumed, this, _1, _2, _3);

  this->mqtt_connection_->Connect(cert_data["clientID"].get<std::string>().c_str(), false, 1000);
}

void MqttTelemetryPublisher::connectUsingDiscovery(const nlohmann::json &)
{

}

void MqttTelemetryPublisher::onConnectionCompleted(
  Aws::Crt::Mqtt::MqttConnection &, int error_code, Aws::Crt::Mqtt::ReturnCode return_code, bool)
{
  if (error_code) {
    RCLCPP_ERROR(
      this->get_logger(), "Connection failed with error %s",
      Aws::Crt::ErrorDebugString(error_code));
    std::exit(-1);
  } else {
    RCLCPP_INFO(this->get_logger(), "Connection completed with return code %d", return_code);
    this->initSubscription();
  }
}

void MqttTelemetryPublisher::onInterrupted(Aws::Crt::Mqtt::MqttConnection &, int error)
{
  RCLCPP_ERROR(
    this->get_logger(), "Connection interrupted with error %s",
    Aws::Crt::ErrorDebugString(error));
}

void MqttTelemetryPublisher::onResumed(
  Aws::Crt::Mqtt::MqttConnection &, Aws::Crt::Mqtt::ReturnCode, bool)
{
  RCLCPP_INFO(this->get_logger(), "Connection resumed");
}

void MqttTelemetryPublisher::onDisconnect(Aws::Crt::Mqtt::MqttConnection &)
{
  RCLCPP_INFO(this->get_logger(), "Disconnect completed");
}

void MqttTelemetryPublisher::initSubscription()
{
  this->subscription_ = this->create_subscription<std_msgs::msg::String>(
    "mock_telemetry", 10,
    std::bind(&MqttTelemetryPublisher::onSubscriptionMsg, this, std::placeholders::_1));
}

void MqttTelemetryPublisher::onSubscriptionMsg(const std_msgs::msg::String & msg)
{
  Aws::Crt::String message = msg.data.c_str();
  Aws::Crt::ByteBuf payload = Aws::Crt::ByteBufFromArray(
    (const uint8_t *)message.data(), message.length());
  auto onPublishComplete = [](Aws::Crt::Mqtt::MqttConnection &, uint16_t, int) {};
  this->mqtt_connection_->Publish(
    "ros2_mock_telemetry_topic",
    AWS_MQTT_QOS_AT_LEAST_ONCE,
    false,
    payload,
    onPublishComplete);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MqttTelemetryPublisher>());
  rclcpp::shutdown();
  return 0;
}
