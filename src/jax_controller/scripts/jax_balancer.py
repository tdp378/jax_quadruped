#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class JaxBalancer(Node):
    def __init__(self):
        super().__init__('jax_balancer')
        
        # 1. Subscriber to IMU
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        
        # 2. Publisher to the Legs
        self.traj_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        
        # Joint names matching your XACRO
        self.joint_names = [
            'fl_hip_joint', 'fl_thigh_joint', 'fl_calf_joint',
            'fr_hip_joint', 'fr_thigh_joint', 'fr_calf_joint',
            'rl_hip_joint', 'rl_thigh_joint', 'rl_calf_joint',
            'rr_hip_joint', 'rr_thigh_joint', 'rr_calf_joint'
        ]
        self.get_logger().info("JAX Balancer Node Started!")

    def imu_callback(self, msg):
        # 1. Calculate Pitch
        y = msg.orientation.y
        w = msg.orientation.w
        pitch = 2.0 * math.asin(y)
        
        # 2. Add a Deadzone (Prevents "stuck" jittering)
        if abs(pitch) < 0.001:
            pitch = 0.0

        # 3. Create Message
        traj = JointTrajectory()
        # Header is KEY: Tells the controller to execute this NOW
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        adjustment = pitch * 1.2  # Strength of the reaction
        
        # Base stance + adjustment
        f_thigh = 0.6 - adjustment
        r_thigh = 0.6 + adjustment
        
        point.positions = [
            0.0, f_thigh, -1.2,  # FL
            0.0, f_thigh, -1.2,  # FR
            0.0, r_thigh, -1.2,  # RL
            0.0, r_thigh, -1.2   # RR
        ]
        
        # Set a very small time (0.05s) so it feels like a live update
        point.time_from_start.nanosec = 50000000 
        
        traj.points.append(point)
        self.traj_pub.publish(traj)

def main(args=None):
    rclpy.init(args=args)
    node = JaxBalancer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()