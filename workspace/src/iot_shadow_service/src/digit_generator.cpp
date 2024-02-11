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

#include <iot_shadow_service/digit_generator.hpp>

DigitGenerator::DigitGenerator()
: rclcpp::Node("digit_generator")
{
  this->init_timer_ = this->create_wall_timer(
    std::chrono::seconds(0),
    std::bind(&DigitGenerator::init, this));
}

void DigitGenerator::init()
{
  this->init_timer_.reset();

  this->update_shadow_cli_ =
    this->create_client<iot_shadow_service_msgs::srv::UpdateShadow>("publish_to_shadow");

  while (!this->update_shadow_cli_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }

  this->generate_digi_timer_ = this->create_wall_timer(
    std::chrono::seconds(4),
    std::bind(&DigitGenerator::generateDigit, this));
}

void DigitGenerator::generateDigit()
{
  static std::default_random_engine generator;
  static std::uniform_int_distribution<int> distribution(1, 100);
  int next_digit = distribution(generator);

  auto request = std::make_shared<iot_shadow_service_msgs::srv::UpdateShadow::Request>();
  nlohmann::json j;
  j["digit"] = next_digit;
  request->desired = j.dump();

  RCLCPP_INFO(this->get_logger(), "Next digit: %d", next_digit);

  this->update_shadow_cli_->async_send_request(
    std::move(request),
    [this](rclcpp::Client<iot_shadow_service_msgs::srv::UpdateShadow>::SharedFuture future) {
      std::string result = future.get()->success ? "Success" : "Failure";
      RCLCPP_INFO(this->get_logger(), "Result of service call: %s", result.c_str());
    }
  );
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DigitGenerator>());
  rclcpp::shutdown();
  return 0;
}
