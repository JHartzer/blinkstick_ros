// Copyright 2022 Jacob Hartzer
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "blinkstick_msgs/msg/color.hpp"
#include <rclcpp/duration.hpp>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class BlinkstickTester : public rclcpp::Node
{
public:
  BlinkstickTester()
  : Node("BlinkstickTester")
  {
    m_publisher = this->create_publisher<blinkstick_msgs::msg::Color>("blinkstick", 10);
    m_timer = this->create_wall_timer(
      500ms, std::bind(&BlinkstickTester::m_timer_callback, this));
  }

private:
  void m_timer_callback()
  {
    RCLCPP_INFO(this->get_logger(), "Timer callback: publishing");
    auto message = blinkstick_msgs::msg::Color();
    message.red = m_colors[0];
    message.green = m_colors[1];
    message.blue = m_colors[2];
    m_publisher->publish(message);
    std::rotate(m_colors.begin(), m_colors.begin() + 1, m_colors.end());
  }

  rclcpp::TimerBase::SharedPtr m_timer;
  rclcpp::Publisher<blinkstick_msgs::msg::Color>::SharedPtr m_publisher;
  std::vector<unsigned int> m_colors {255, 0, 0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BlinkstickTester>());
  rclcpp::shutdown();
  return 0;
}
