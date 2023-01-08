#include <iostream>
#include <memory>

#include "blinkstick_msgs/msg/blink.hpp"
#include "blinkstick/device.hpp"
#include "blinkstick/blinkstick.hpp"


int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  std::cout << "hello world blink stick package" << std::endl;
  return 0;
}
