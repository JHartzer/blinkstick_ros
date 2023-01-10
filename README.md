# blinkstick_ros

A c++ ROS driver for the [blinkstick](https://www.blinkstick.com/) built on the [blinkstickcpp](https://github.com/teekae/blinkstickcpp) package.

## Requirements
- [ROS](https://docs.ros.org/en/humble/index.html)
- [hidapi](https://github.com/signal11/hidapi)
- [cmake](https://cmake.org/)

## UDEV Rules

To not run node as sudo, add the following lines to `/etc/udev/rules.d/99-usb-serial.rules`
```
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="20a0", ATTRS{idProduct}=="41e5", MODE="0660", GROUP="plugdev"
SUBSYSTEM=="usb",    ATTRS{idVendor}=="20a0", ATTRS{idProduct}=="41e5", MODE="0660", GROUP="plugdev"
```