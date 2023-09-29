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

#include <telemetry_mqtt/mock_telemetry_publisher.hpp>

MockTelemetryPublisher::MockTelemetryPublisher()
: rclcpp::Node("mock_telemetry_publisher"),
  random_engine(std::random_device{}())
{
  this->publisher_ = this->create_publisher<std_msgs::msg::String>(
    "mock_telemetry", 10);

  using namespace std::chrono_literals;
  this->timer_ = this->create_wall_timer(
    0.5s, std::bind(&MockTelemetryPublisher::TimerCallback, this));
}

void MockTelemetryPublisher::TimerCallback()
{
  std_msgs::msg::String msg;
  std::uniform_real_distribution<double> battery_distribution(85.0, 90.0);
  std::uniform_real_distribution<double> velocity_distribution(3, 4);

  std::ostringstream stream;
  stream << std::fixed << std::setprecision(2)
         << "{\"battery\": "
         << battery_distribution(this->random_engine)
         << ", \"velocity\": "
         << velocity_distribution(this->random_engine)
         << "}";

  msg.data = stream.str();
  this->publisher_->publish(msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MockTelemetryPublisher>());
  rclcpp::shutdown();
  return 0;
}
