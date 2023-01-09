# blinkstick_ros

A ROS driver for the blinkstick


Add the following lines to `/etc/udev/rules.d/99-usb-serial.rules`
```
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="20a0", ATTRS{idProduct}=="41e5", MODE="0660", GROUP="plugdev"
SUBSYSTEM=="usb",    ATTRS{idVendor}=="20a0", ATTRS{idProduct}=="41e5", MODE="0660", GROUP="plugdev"
```