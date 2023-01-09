#include <iostream>
#include <memory>

#include "blinkstick_msgs/msg/blink.hpp"
#include "blinkstick/device.hpp"
#include "blinkstick/blinkstick.hpp"

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    RCLCPP_INFO(this->get_logger(), "Initialize Node");

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

    // int args_red = 255;
    // int args_green = 255;
    // int args_blue = 0;
    // int args_index = 1;
    blinkstick::enable_logging();
    m_device = blinkstick::find();
    if (!m_device.is_valid()) {
      RCLCPP_ERROR(this->get_logger(), "No connected BlinkStick");
      rclcpp::shutdown();
    }

    m_led_count = m_device.get_led_count();
    std::cout << "There are " << m_led_count << " LEDs\n";

    // set the color
    bool red = true;
    m_colours.resize(m_led_count);
    for (int i = 0; i < m_led_count; ++i) {
      auto & colour = m_colours[i];

      if (red) {
        //colour.red = 255;
        colour.red = 255;
        colour.green = 215;
      } else {
        //colour.green = 255;
        colour.red = 64;
        colour.green = 219;
        colour.blue = 219;
      }
      red = !red;
    }
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    m_device.set_colours(m_args_channel, m_colours);
    // std::rotate(m_colours.begin(), m_colours.begin() + 1, m_colours.end());
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  int m_led_count {0};
  std::vector<blinkstick::colour> m_colours;
  int m_args_channel {0};
  blinkstick::device m_device = blinkstick::device(nullptr, blinkstick::device_type::unknown);

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  blinkstick::finalise();
  return 0;
}
