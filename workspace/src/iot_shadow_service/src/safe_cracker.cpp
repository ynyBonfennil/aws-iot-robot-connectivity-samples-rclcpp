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

#include <iot_shadow_service/safe_cracker.hpp>

SafeCracker::SafeCracker()
: rclcpp::Node("safe_cracker")
{
  this->init_timer_ = this->create_wall_timer(
    std::chrono::seconds(0),
    std::bind(&SafeCracker::init, this));
}

void SafeCracker::init()
{
  this->init_timer_.reset();

  this->current_digit_ = 0;
  this->target_digit_ = 0;
  this->clockwise_ = true;

  this->shadow_update_snapshot_sub_ =
    this->create_subscription<iot_shadow_service_msgs::msg::ShadowUpdateSnapshot>(
    "shadow_update_snapshot",
    rclcpp::QoS(1).best_effort().durability_volatile(),
    std::bind(&SafeCracker::onShadowUpdateSnapshot, this, std::placeholders::_1));

  this->update_shadow_cli_ =
    this->create_client<iot_shadow_service_msgs::srv::UpdateShadow>("publish_to_shadow");
  while (!this->update_shadow_cli_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }

  this->spin_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&SafeCracker::turnTowardsTarget, this));
}

void SafeCracker::onShadowUpdateSnapshot(
  const iot_shadow_service_msgs::msg::ShadowUpdateSnapshot & msg)
{
  nlohmann::json j = nlohmann::json::parse(msg.desired);
  RCLCPP_INFO(this->get_logger(), "Desired shadow update is %s", msg.desired.c_str());
  int desired_digit = j.value("digit", 0);

  if (desired_digit != this->target_digit_) {
    this->target_digit_ = desired_digit;
    this->clockwise_ = !this->clockwise_;

    std::string direction_str = this->clockwise_ ? "clockwise" : "anti-clockwise";
    RCLCPP_INFO(
      this->get_logger(), "Trying to reach %d from %d spinning %s",
      this->target_digit_, this->current_digit_, direction_str.c_str());
  }
}

void SafeCracker::turnTowardsTarget()
{
  int direction_mult = this->clockwise_ ? 1 : -1;
  int delta = (direction_mult * (this->target_digit_ - this->current_digit_)) % this->kDIGITS_ON_DIAL;
  int change = delta < this->kMAX_SPEED ? delta : kMAX_SPEED;
  if (change == 0) {
    return;
  }

  this->current_digit_ = (this->current_digit_ + change) % this->kDIGITS_ON_DIAL;
  std::string direction_str = this->clockwise_ ? "clockwise" : "anti-clockwise";
  RCLCPP_INFO(this->get_logger(), "Trying to reach %d from %d spinning %s",
  this->target_digit_, this->current_digit_, direction_str.c_str());

  auto update_target_request = std::make_shared<iot_shadow_service_msgs::srv::UpdateShadow::Request>();
  nlohmann::json j;
  j["digit"] = this->current_digit_;
  update_target_request->reported = j.dump();
  RCLCPP_INFO(this->get_logger(), "Updating target digit with message: %s", update_target_request->reported.c_str());
  this->update_shadow_cli_->async_send_request(std::move(update_target_request));
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafeCracker>());
  rclcpp::shutdown();
  return 0;
}
