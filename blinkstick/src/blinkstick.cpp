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

#include <iostream>
#include <memory>

#include "blinkstick_msgs/msg/color_array.hpp"
#include "blinkstick/device.hpp"
#include "blinkstick/blinkstick.hpp"

#include "rclcpp/rclcpp.hpp"
using std::placeholders::_1;

class BlinkstickNode : public rclcpp::Node
{
public:
  BlinkstickNode()
  : Node("blinkstick")
  {
    RCLCPP_INFO(this->get_logger(), "Initialize Node");

    m_subscriber = this->create_subscription<blinkstick_msgs::msg::ColorArray>(
      "blinkstick", 10, std::bind(&BlinkstickNode::topic_callback, this, _1));

    blinkstick::enable_logging();
    m_device = blinkstick::find();
    if (!m_device.is_valid()) {
      RCLCPP_ERROR(this->get_logger(), "No connected BlinkStick");
      rclcpp::shutdown();
    }

    m_led_count = m_device.get_led_count();
    rclcpp::on_shutdown(std::bind(&BlinkstickNode::shutdown, this));
  }

private:
  void topic_callback(const blinkstick_msgs::msg::ColorArray::SharedPtr msg) const
  {
    std::vector<blinkstick::colour> colors(m_led_count);
    for (unsigned int i = 0; (i < colors.size() && i < msg.get()->colors.size()); ++i) {
      colors[i].red = msg.get()->colors[i].r;
      colors[i].green = msg.get()->colors[i].g;
      colors[i].blue = msg.get()->colors[i].b;
    }
    m_device.set_colours(m_args_channel, colors);
  }

  void shutdown()
  {
    m_device.off();
  }

  rclcpp::Subscription<blinkstick_msgs::msg::ColorArray>::SharedPtr m_subscriber;
  int m_led_count {0};
  int m_args_channel {0};
  blinkstick::device m_device = blinkstick::device(nullptr, blinkstick::device_type::unknown);
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BlinkstickNode>());
  rclcpp::shutdown();
  return 0;
}
