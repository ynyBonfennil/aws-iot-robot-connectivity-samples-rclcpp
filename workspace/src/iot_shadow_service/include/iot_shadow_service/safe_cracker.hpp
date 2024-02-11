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
#include <memory>
#include <nlohmann/json.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>

#include <iot_shadow_service_msgs/msg/shadow_update_snapshot.hpp>
#include <iot_shadow_service_msgs/srv/update_shadow.hpp>

class SafeCracker : public rclcpp::Node
{
private:
  const int kDIGITS_ON_DIAL = 100;
  const int kMAX_SPEED = 3;

  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr spin_timer_;
  rclcpp::Client<iot_shadow_service_msgs::srv::UpdateShadow>::SharedPtr
    update_shadow_cli_;
  rclcpp::Subscription<iot_shadow_service_msgs::msg::ShadowUpdateSnapshot>::SharedPtr
    shadow_update_snapshot_sub_;

  int current_digit_, target_digit_;
  bool clockwise_;

public:
  SafeCracker();
  void onShadowUpdateSnapshot(const iot_shadow_service_msgs::msg::ShadowUpdateSnapshot &);
  void turnTowardsTarget();

private:
  void init();
};
