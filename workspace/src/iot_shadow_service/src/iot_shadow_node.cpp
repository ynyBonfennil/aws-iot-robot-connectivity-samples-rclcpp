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

#include <iot_shadow_service/iot_shadow_node.hpp>

IoTShadowNode::IoTShadowNode()
: rclcpp::Node("IoT_shadow_node")
{
  this->declare_parameter<std::string>("path_for_config", "");
  this->declare_parameter<std::string>("shadow_name", "");
  this->declare_parameter<bool>("discover_endpoints", false);

  this->get_parameter("path_for_config", this->path_for_config_);
  this->get_parameter("shadow_name", this->shadow_name_);
  this->get_parameter("discover_endpoints", this->discover_endpoints_);

  this->init_timer_ = this->create_wall_timer(
    std::chrono::seconds(0),
    std::bind(&IoTShadowNode::initMqttConnection, this));
}

IoTShadowNode::~IoTShadowNode()
{
  this->mqtt_connection_->Disconnect();
}

void IoTShadowNode::initMqttConnection()
{
  this->init_timer_.reset();

  std::ifstream config_file(this->path_for_config_);
  nlohmann::json cert_data;
  config_file >> cert_data;
  RCLCPP_INFO(this->get_logger(), "Config we are loading is :\n%s", cert_data.dump().c_str());
  this->thing_name_ = cert_data["clientID"].get<std::string>();

  // Establish MQTT Connection
  if (this->discover_endpoints_) {
    RCLCPP_INFO(this->get_logger(), "Discovering endpoints for connection");
    this->connectUsingDiscovery(cert_data);
  } else {
    RCLCPP_INFO(this->get_logger(), "Connecting directly to endpoint");
    this->connectToEndpoint(cert_data);
  }
}

void IoTShadowNode::connectToEndpoint(const nlohmann::json & cert_data)
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
    &IoTShadowNode::onConnectionCompleted, this, _1, _2, _3, _4);
  this->mqtt_connection_->OnDisconnect = std::bind(
    &IoTShadowNode::onDisconnect, this, _1);
  this->mqtt_connection_->OnConnectionInterrupted = std::bind(
    &IoTShadowNode::onInterrupted, this, _1, _2);
  this->mqtt_connection_->OnConnectionResumed = std::bind(
    &IoTShadowNode::onResumed, this, _1, _2, _3);

  this->mqtt_connection_->Connect(cert_data["clientID"].get<std::string>().c_str(), false, 1000);
}

void IoTShadowNode::connectUsingDiscovery(const nlohmann::json &)
{

}

void IoTShadowNode::onConnectionCompleted(
  Aws::Crt::Mqtt::MqttConnection &, int error_code, Aws::Crt::Mqtt::ReturnCode return_code, bool)
{
  if (error_code) {
    RCLCPP_ERROR(
      this->get_logger(), "Connection failed with error %s",
      Aws::Crt::ErrorDebugString(error_code));
    std::exit(-1);
  } else {
    RCLCPP_INFO(this->get_logger(), "Connection completed with return code %d", return_code);
    this->setupDeviceShadow();
    this->initNodeInterfaces();
  }
}

void IoTShadowNode::onInterrupted(Aws::Crt::Mqtt::MqttConnection &, int error)
{
  RCLCPP_ERROR(
    this->get_logger(), "Connection interrupted with error %s",
    Aws::Crt::ErrorDebugString(error));
}

void IoTShadowNode::onResumed(
  Aws::Crt::Mqtt::MqttConnection &, Aws::Crt::Mqtt::ReturnCode, bool)
{
  RCLCPP_INFO(this->get_logger(), "Connection resumed");
}

void IoTShadowNode::onDisconnect(Aws::Crt::Mqtt::MqttConnection &)
{
  RCLCPP_INFO(this->get_logger(), "Disconnect completed");
}

void IoTShadowNode::setupDeviceShadow()
{
  // Setup device shadow connection
  this->subscribe_delta_completed_promise_ = std::make_shared<std::promise<void>>();
  this->subscribe_delta_accepted_completed_promise_ = std::make_shared<std::promise<void>>();
  this->subscribe_delta_rejected_completed_promise_ = std::make_shared<std::promise<void>>();

  Aws::Iotshadow::IotShadowClient shadow_client(this->mqtt_connection_);
  Aws::Iotshadow::ShadowDeltaUpdatedSubscriptionRequest shadow_delta_updated_request;
  shadow_delta_updated_request.ThingName = this->thing_name_;

  using namespace std::placeholders;
  shadow_client.SubscribeToShadowDeltaUpdatedEvents(
    shadow_delta_updated_request,
    AWS_MQTT_QOS_AT_LEAST_ONCE,
    std::bind(&IoTShadowNode::onDeltaUpdated, this, _1, _2),
    std::bind(&IoTShadowNode::onDeltaUpdatedSubAck, this, _1));
  
  Aws::Iotshadow::UpdateShadowSubscriptionRequest update_shadow_subscription_request;
  update_shadow_subscription_request.ThingName = this->thing_name_;

  shadow_client.SubscribeToUpdateShadowAccepted(
    update_shadow_subscription_request,
    AWS_MQTT_QOS_AT_LEAST_ONCE,
    std::bind(&IoTShadowNode::onUpdateShadowAccepted, this, _1, _2),
    std::bind(&IoTShadowNode::onDeltaUpdatedRejectedSubAck, this, _1));
  
  shadow_client.SubscribeToUpdateShadowRejected(
    update_shadow_subscription_request,
    AWS_MQTT_QOS_AT_LEAST_ONCE,
    std::bind(&IoTShadowNode::onUpdateShadowRejected, this, _1, _2),
    std::bind(&IoTShadowNode::onDeltaUpdatedRejectedSubAck, this, _1));

  this->subscribe_delta_completed_promise_->get_future().wait();
  this->subscribe_delta_accepted_completed_promise_->get_future().wait();
  this->subscribe_delta_rejected_completed_promise_->get_future().wait();

  // Get initial shadow document
  this->subscribe_get_shadow_accepted_completed_promise_ = std::make_shared<std::promise<void>>();
  this->subscribe_get_shadow_rejected_completed_promise_ = std::make_shared<std::promise<void>>();
  Aws::Iotshadow::GetShadowSubscriptionRequest shadow_subscription_request;
  shadow_subscription_request.ThingName = this->thing_name_;
  shadow_client.SubscribeToGetShadowAccepted(
    shadow_subscription_request,
    AWS_MQTT_QOS_AT_LEAST_ONCE,
    std::bind(&IoTShadowNode::onGetShadowAccepted, this, _1, _2),
    std::bind(&IoTShadowNode::onGetShadowUpdatedAcceptedSubAck, this, _1));
  
  shadow_client.SubscribeToGetShadowRejected(
    shadow_subscription_request,
    AWS_MQTT_QOS_AT_LEAST_ONCE,
    std::bind(&IoTShadowNode::onGetShadowRejected, this, _1, _2),
    std::bind(&IoTShadowNode::onGetShadowUpdatedRejectedSubAck, this, _1));
  
  this->subscribe_get_shadow_accepted_completed_promise_->get_future().wait();
  this->subscribe_get_shadow_rejected_completed_promise_->get_future().wait();

  this->on_get_shadow_request_completed_promise_ = std::make_shared<std::promise<void>>();
  this->got_initial_shadow_promise_ = std::make_shared<std::promise<void>>();

  Aws::Iotshadow::GetShadowRequest shadow_get_request;
  shadow_get_request.ThingName = this->thing_name_;
  shadow_client.PublishGetShadow(
    shadow_get_request,
    AWS_MQTT_QOS_AT_LEAST_ONCE,
    std::bind(&IoTShadowNode::onGetShadowRequestSubAck, this, _1));
  
  this->on_get_shadow_request_completed_promise_->get_future().wait();
  this->got_initial_shadow_promise_->get_future().wait();
}

void IoTShadowNode::onDeltaUpdatedSubAck(int error)
{
  if (error != AWS_OP_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Error subscribing to shadow delta: %s", Aws::Crt::ErrorDebugString(error));
    return;
  }
  this->subscribe_delta_completed_promise_->set_value();
}

void IoTShadowNode::onDeltaUpdatedAcceptedSubAck(int error)
{
  if (error != AWS_OP_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Error subscribing to shadow delta accepted: %s", Aws::Crt::ErrorDebugString(error));
    return;
  }
  this->subscribe_delta_accepted_completed_promise_->set_value();
}

void IoTShadowNode::onDeltaUpdatedRejectedSubAck(int error)
{
  if (error != AWS_OP_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Error subscribing to shadow delta rejected: %s", Aws::Crt::ErrorDebugString(error));
    return;
  }
  this->subscribe_delta_rejected_completed_promise_->set_value();
}

void IoTShadowNode::onDeltaUpdated(Aws::Iotshadow::ShadowDeltaUpdatedEvent *, int )
{

}

void IoTShadowNode::onUpdateShadowAccepted(Aws::Iotshadow::UpdateShadowResponse *, int)
{

}

void IoTShadowNode::onUpdateShadowRejected(Aws::Iotshadow::ErrorResponse *, int)
{

}

void IoTShadowNode::onGetShadowUpdatedAcceptedSubAck(int error)
{
  if (error != AWS_OP_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Error subscribing to get shadow document accepted: %s", Aws::Crt::ErrorDebugString(error));
    return;
  }
  this->subscribe_get_shadow_accepted_completed_promise_->set_value();
}

void IoTShadowNode::onGetShadowUpdatedRejectedSubAck(int error)
{
  if (error != AWS_OP_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Error subscribing to get shadow document rejected: %s", Aws::Crt::ErrorDebugString(error));
    return;
  }
  this->subscribe_get_shadow_rejected_completed_promise_->set_value();
}

void IoTShadowNode::onGetShadowRequestSubAck(int error)
{
  if (error != AWS_OP_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Error getting shadow document: %s", Aws::Crt::ErrorDebugString(error));
    return;
  }
  this->on_get_shadow_request_completed_promise_->set_value();
}

void IoTShadowNode::onGetShadowAccepted(Aws::Iotshadow::GetShadowResponse *, int)
{

}

void IoTShadowNode::onGetShadowRejected(Aws::Iotshadow::ErrorResponse *, int)
{

}

void IoTShadowNode::initNodeInterfaces()
{
  using namespace std::placeholders;
  this->publish_to_shadow_srv_ = this->create_service<iot_shadow_service_msgs::srv::UpdateShadow>(
    "publish_to_shadow",
    std::bind(&IoTShadowNode::onPublishToShadow, this, _1, _2));
  this->shadow_update_snapshot_pub_ = this->create_publisher<iot_shadow_service_msgs::msg::ShadowUpdateSnapshot>(
    "shadow_update_snapshot",
    rclcpp::QoS(0).reliable().durability_volatile());
}

void IoTShadowNode::onShadowUpdateSnapshot(const iot_shadow_service_msgs::msg::ShadowUpdateSnapshot & msg) {
  RCLCPP_INFO(this->get_logger(), "Got update with contents:");
  RCLCPP_INFO(this->get_logger(), "desired: %s", msg.desired.c_str());
  RCLCPP_INFO(this->get_logger(), "reported: %s", msg.reported.c_str());

  auto pub_msg = std::make_unique<iot_shadow_service_msgs::msg::ShadowUpdateSnapshot>();
  pub_msg->desired = msg.desired;
  pub_msg->reported = msg.reported;
  this->shadow_update_snapshot_pub_->publish(std::move(pub_msg));
  RCLCPP_INFO(this->get_logger(), "Published message:");
  RCLCPP_INFO(this->get_logger(), "desired: %s", pub_msg->desired.c_str());
  RCLCPP_INFO(this->get_logger(), "reported: %s", pub_msg->reported.c_str());
}

void IoTShadowNode::onPublishToShadow(
  const std::shared_ptr<iot_shadow_service_msgs::srv::UpdateShadow::Request> request,
  std::shared_ptr<iot_shadow_service_msgs::srv::UpdateShadow::Response>)
{
  RCLCPP_INFO(this->get_logger(), "Got publish shadow request:");
  RCLCPP_INFO(this->get_logger(), "desired: %s", request->desired.c_str());
  RCLCPP_INFO(this->get_logger(), "reported: %s", request->reported.c_str());

  // shadow state

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IoTShadowNode>());
  rclcpp::shutdown();
  return 0;
}
