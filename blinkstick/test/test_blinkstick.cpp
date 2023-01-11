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
#include "blinkstick_msgs/msg/color_array.hpp"
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
    m_publisher = this->create_publisher<blinkstick_msgs::msg::ColorArray>("blinkstick", 10);
    m_timer = this->create_wall_timer(
      500ms, std::bind(&BlinkstickTester::m_timer_callback, this));
  }

private:
  void m_timer_callback()
  {
    auto color_array = blinkstick_msgs::msg::ColorArray();
    auto color = std_msgs::msg::ColorRGBA();
    for (unsigned int i = 0; i < 2; ++i) {
      color.a = 1.0;
      color.r = m_colors[0];
      color.g = m_colors[1];
      color.b = m_colors[2];
      color_array.colors.push_back(color);
      std::rotate(m_colors.begin(), m_colors.begin() + 1, m_colors.end());
    }
    m_publisher->publish(color_array);
    RCLCPP_INFO(this->get_logger(), "Publishing Color Array");
  }

  rclcpp::TimerBase::SharedPtr m_timer;
  rclcpp::Publisher<blinkstick_msgs::msg::ColorArray>::SharedPtr m_publisher;
  std::vector<unsigned int> m_colors {255, 0, 0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BlinkstickTester>());
  rclcpp::shutdown();
  return 0;
}
