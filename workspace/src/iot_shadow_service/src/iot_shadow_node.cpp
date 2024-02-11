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

  this->init_timer_ = this->create_wall_timer(
    std::chrono::seconds(0),
    std::bind(&IoTShadowNode::init, this));
}

void IoTShadowNode::init()
{
  this->init_timer_.reset();

  // Load parameters
  std::string path_for_config = this->get_parameter("path_for_config").as_string();
  std::ifstream input_file(path_for_config);
  nlohmann::json j;
  if (input_file.is_open()) {
    input_file >> j;
    input_file.close();
    RCLCPP_INFO(this->get_logger(), "Config we are loading is: %s", j.dump(4).c_str());
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to open path_for_config file");
    return;
  }
  this->thing_name_ = j.value("clientID", "");
  this->shadow_name_ = this->get_parameter("shadow_name").as_string();

  // Build MQTT Connection

  // Create connection to IoT Shadow

  // Create ROS2 service server and publisher
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
  std::shared_ptr<iot_shadow_service_msgs::srv::UpdateShadow::Response> response)
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
