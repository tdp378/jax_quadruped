#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math

class JaxStandNode(Node):
    def __init__(self):
        super().__init__('jax_stand_node')
        
        # Publisher for the Joint Trajectory Controller
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10
        )
        
        # Timer running at 20Hz (0.05s) for smooth animation
        self.timer = self.create_timer(0.05, self.update_motion)
        
        # Start time for sine wave calculations
        self.start_time = self.get_clock().now()
        
        self.get_logger().info('JAX is online. Parallel Sway and Breathing enabled.')

    def update_motion(self):
        # Calculate elapsed time in seconds
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        
        # --- MOTION CALCULATIONS ---
        
        # 1. Lateral Sway (Hips) - 0.15 rad amplitude
        sway = 0.15 * math.sin(elapsed * 1.5)
        
        # 2. Vertical Breathing (Thighs/Calves)
        # We oscillate around the "Stand" pose (Thigh: 0.4, Calf: -0.6)
        bounce = 0.08 * math.sin(elapsed * 1.0) 
        thigh_pos = 0.4 + bounce
        calf_pos = -0.6 - (bounce * 1.2) # Calf compensates to keep feet flat

        # --- MESSAGE CONSTRUCTION ---

        msg = JointTrajectory()
        msg.joint_names = [
            'fl_hip_joint', 'fl_thigh_joint', 'fl_calf_joint',
            'fr_hip_joint', 'fr_thigh_joint', 'fr_calf_joint',
            'rl_hip_joint', 'rl_thigh_joint', 'rl_calf_joint',
            'rr_hip_joint', 'rr_thigh_joint', 'rr_calf_joint'
        ]

        point = JointTrajectoryPoint()
        
        # Applying the HIP FIX: 
        # Front hips use +sway, Rear hips use -sway to maintain parallel motion
        point.positions = [
            sway,  thigh_pos, calf_pos, # Front Left
            sway,  thigh_pos, calf_pos, # Front Right
            -sway, thigh_pos, calf_pos, # Rear Left (Flipped for symmetry)
            -sway, thigh_pos, calf_pos  # Rear Right (Flipped for symmetry)
        ]
        
        # Set the transition time to match our timer (50ms)
        point.time_from_start = Duration(sec=0, nanosec=50000000)

        msg.points.append(point)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JaxStandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('JAX Node stopping...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()