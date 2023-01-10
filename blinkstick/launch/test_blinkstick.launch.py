#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='blinkstick',
            executable='blinkstick'
        ),
        Node(
            package='blinkstick',
            executable='blinkstick_test'
        )
    ])
