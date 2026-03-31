#!/usr/bin/env python3
"""
fetch_urdf_links.py
Fetches all links from the robot URDF and prints them
in YAML format for use in the arm footprint params.

Usage:
    ros2 run cs4mt_armfootprint_aware_navigation fetch_urdf_links.py

Or directly:
    python3 fetch_urdf_links.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import xml.etree.ElementTree as ET


class URDFLinkFetcher(Node):

    def __init__(self):
        super().__init__('urdf_link_fetcher')

        # Keywords to exclude from arm links
        self.exclude_keywords = [
            'base', 'laser', 'camera', 'imu',
            'wheel', 'caster', 'sonar', 'odom',
            'footprint', 'world', 'map', 'scan',
            'cliff', 'bumper', 'dock'
        ]

        # Subscribe to robot description
        self.subscription = self.create_subscription(
            String,
            '/robot_description',
            self.urdf_callback,
            rclpy.qos.QoSProfile(
                depth=1,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
            )
        )
        self.get_logger().info('Waiting for /robot_description...')

    def urdf_callback(self, msg):
        try:
            root = ET.fromstring(msg.data)
        except ET.ParseError as e:
            self.get_logger().error(f'Failed to parse URDF: {e}')
            return

        # Get all link names
        all_links = [link.get('name') for link in root.findall('link')]

        # Filter out excluded keywords
        arm_links = [
            link for link in all_links
            if not any(kw in link.lower() for kw in self.exclude_keywords)
            and link is not None
        ]

        # Print all links found
        self.get_logger().info(f'Total links in URDF: {len(all_links)}')
        self.get_logger().info(f'Filtered arm links: {len(arm_links)}')

        # Print full link list for reference
        print('\n--- ALL URDF LINKS ---')
        for link in all_links:
            print(f'  {link}')

        # Print filtered arm links in YAML format
        print('\n--- YAML CONFIG (paste into params.yaml) ---')
        print('arm_links:')
        for link in arm_links:
            print(f'  - "{link}"')

        print('\nlink_radii:')
        for _ in arm_links:
            print(f'  - 0.08  # adjust per link')

        print('\n--- VERIFY LINKS IN TF TREE ---')
        print('Run this to verify each link exists in TF:')
        for link in arm_links:
            print(f'  ros2 run tf2_ros tf2_echo base_link {link}')

        rclpy.shutdown()


def main():
    rclpy.init()
    node = URDFLinkFetcher()
    rclpy.spin(node)


if __name__ == '__main__':
    main()