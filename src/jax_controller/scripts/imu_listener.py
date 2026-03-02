#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class ImuListener(Node):
    def __init__(self):
        super().__init__('imu_listener')
        # Subscribe to the /imu topic created by the ros_gz_bridge
        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10) # QoS profile

    def imu_callback(self, msg):
        # 1. Get Quaternion data
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w

        # 2. Convert to Euler Angles (Roll, Pitch, Yaw)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.degrees(math.asin(t2))

        # 3. Print the balance state
        self.get_logger().info(f'Roll: {roll:.2f}°, Pitch: {pitch:.2f}°')

def main(args=None):
    rclpy.init(args=args)
    node = ImuListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()