#!/usr/bin/env python3

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_srvs.srv import Empty


class AutoGlobalLocalizer(Node):
    def __init__(self):
        super().__init__('auto_global_localizer')

        self.declare_parameter('start_delay', 5.0)
        self.declare_parameter('spin_duration', 18.0)
        self.declare_parameter('angular_speed', 0.2)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('global_localization_service', '/reinitialize_global_localization')

        self.start_delay = float(self.get_parameter('start_delay').value)
        self.spin_duration = float(self.get_parameter('spin_duration').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.service_name = self.get_parameter('global_localization_service').value

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.global_localization_client = self.create_client(Empty, self.service_name)

    def run(self):
        self.get_logger().info(
            f'Waiting {self.start_delay:.1f}s before global localization'
        )
        time.sleep(self.start_delay)

        self.get_logger().info(f'Waiting for {self.service_name}')
        if not self.global_localization_client.wait_for_service(timeout_sec=30.0):
            self.get_logger().error(
                f'Service {self.service_name} is not available; aborting auto localization'
            )
            return

        self.get_logger().info('Calling AMCL global localization')
        future = self.global_localization_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is None:
            self.get_logger().error('Global localization service call failed')
            return

        self.get_logger().info(
            f'Spinning in place for {self.spin_duration:.1f}s at {self.angular_speed:.2f} rad/s'
        )
        twist = Twist()
        twist.angular.z = self.angular_speed

        start_time = time.monotonic()
        rate_hz = 10.0
        interval = 1.0 / rate_hz
        while rclpy.ok() and time.monotonic() - start_time < self.spin_duration:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(interval)

        self.stop_robot()
        self.get_logger().info('Auto global localization sequence finished')

    def stop_robot(self):
        stop = Twist()
        for _ in range(10):
            self.cmd_pub.publish(stop)
            time.sleep(0.05)


def main(args=None):
    rclpy.init(args=args)
    node = AutoGlobalLocalizer()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
