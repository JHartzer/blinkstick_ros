#include <iostream>
#include <memory>

#include "blinkstick_msgs/msg/color.hpp"
#include "blinkstick/device.hpp"
#include "blinkstick/blinkstick.hpp"

#include <memory>

#include "rclcpp/rclcpp.hpp"
using std::placeholders::_1;

class BlinkstickNode : public rclcpp::Node
{
public:
  BlinkstickNode()
  : Node("blinkstick")
  {
    RCLCPP_INFO(this->get_logger(), "Initialize Node");

    m_subscriber = this->create_subscription<blinkstick_msgs::msg::Color>(
      "blinkstick", 10, std::bind(&BlinkstickNode::topic_callback, this, _1));

    blinkstick::enable_logging();
    m_device = blinkstick::find();
    if (!m_device.is_valid()) {
      RCLCPP_ERROR(this->get_logger(), "No connected BlinkStick");
      rclcpp::shutdown();
    }

    m_led_count = m_device.get_led_count();
  }

private:
  void topic_callback(const blinkstick_msgs::msg::Color::SharedPtr msg) const
  {


    std::vector<blinkstick::colour> colors(m_led_count);
    std::string log = "Subscriber callback. Size: " + std::to_string(colors.size());
    RCLCPP_INFO(this->get_logger(), log.c_str());
    for (unsigned int i = 0; i < colors.size(); ++i) {
      colors[i].red = msg.get()->red;
      colors[i].green = msg.get()->green;
      colors[i].blue = msg.get()->blue;
    }
    m_device.set_colours(m_args_channel, colors);
  }

  rclcpp::Subscription<blinkstick_msgs::msg::Color>::SharedPtr m_subscriber;
  int m_led_count {0};
  int m_args_channel {0};
  blinkstick::device m_device = blinkstick::device(nullptr, blinkstick::device_type::unknown);
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BlinkstickNode>());
  rclcpp::shutdown();
  blinkstick::finalise();
  return 0;
}
