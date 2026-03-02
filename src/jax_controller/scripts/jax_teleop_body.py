#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys, select, termios, tty
import math

msg = """
JAX Smooth Control
---------------------------
W / S : Lean Forward / Back
A / D : Lean Left / Right
R / F : Change Height
H     : Toggle Wave
K     : Toggle Sit
Space : SMOOTH Reset to Zero
CTRL-C to quit
"""

class JaxTeleopBody(Node):
    def __init__(self):
        super().__init__('jax_teleop_body')
        self.pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        
        # Current Values (What the robot is actually doing)
        self.pitch = 0.0
        self.roll = 0.0
        self.height = 0.0
        
        # Target Values (Where we want to go)
        self.t_pitch = 0.0
        self.t_roll = 0.0
        self.t_height = 0.0
        
        # Animation States
        self.is_waving = False
        self.wave_step = 0.0
        self.is_sitting = False
        self.sit_lerp = 0.0
        
        self.joint_names = [
            'fl_hip_joint', 'fl_thigh_joint', 'fl_calf_joint',
            'fr_hip_joint', 'fr_thigh_joint', 'fr_calf_joint',
            'rl_hip_joint', 'rl_thigh_joint', 'rl_calf_joint',
            'rr_hip_joint', 'rr_thigh_joint', 'rr_calf_joint'
        ]
        
        self.timer = self.create_timer(0.05, self.publish_stance)
        print(msg)

    def get_key(self, settings):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1).lower()
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def publish_stance(self):
        traj = JointTrajectory()
        traj.header.stamp.sec = 0
        traj.header.stamp.nanosec = 0
        traj.joint_names = self.joint_names
        
        # --- 1. THE SMOOTHING ENGINE (The "Universal Lerp") ---
        # This makes current values drift toward targets
        lerp_speed = 0.2  # Adjust 0.05 to 0.2 for speed
        self.pitch += (self.t_pitch - self.pitch) * lerp_speed
        self.roll += (self.t_roll - self.roll) * lerp_speed
        self.height += (self.t_height - self.height) * lerp_speed

        # Smooth Sit Transition
        if self.is_sitting and self.sit_lerp < 1.0:
            self.sit_lerp += 0.05
        elif not self.is_sitting and self.sit_lerp > 0.0:
            self.sit_lerp -= 0.05

        # --- 2. BASE STANCE ---
        fl_t_base = self.height + self.pitch + self.roll
        fr_t_base = self.height + self.pitch - self.roll
        rl_t_base = self.height - self.pitch + self.roll
        rr_t_base = self.height - self.pitch - self.roll

        calf_ratio = -2.5 
        fl_c_base = fl_t_base * calf_ratio
        fr_c_base = fr_t_base * calf_ratio
        rl_c_base = rl_t_base * calf_ratio
        rr_c_base = rr_t_base * calf_ratio

        # --- 3. SIT TARGET POSES ---
        fl_t_sit, fr_t_sit = -0.3, -0.3
        fl_c_sit, fr_c_sit = 0.7, 0.7
        rl_t_sit, rr_t_sit = 0.5, 0.5
        rl_c_sit, rr_c_sit = -0.7, -0.7

        # --- 4. BLENDING ---
        fl_t = fl_t_base * (1 - self.sit_lerp) + fl_t_sit * self.sit_lerp
        fr_t = fr_t_base * (1 - self.sit_lerp) + fr_t_sit * self.sit_lerp
        rl_t = rl_t_base * (1 - self.sit_lerp) + rl_t_sit * self.sit_lerp
        rr_t = rr_t_base * (1 - self.sit_lerp) + rr_t_sit * self.sit_lerp

        fl_c = fl_c_base * (1 - self.sit_lerp) + fl_c_sit * self.sit_lerp
        fr_c = fr_c_base * (1 - self.sit_lerp) + fr_c_sit * self.sit_lerp
        rl_c = rl_c_base * (1 - self.sit_lerp) + rl_c_sit * self.sit_lerp
        rr_c = rr_c_base * (1 - self.sit_lerp) + rr_c_sit * self.sit_lerp
        
        fl_h = fr_h = rl_h = rr_h = 0.0

        # --- 5. WAVE OVERRIDE ---
        if self.is_waving:
            self.wave_step += 0.3
            fl_t = -1.2           
            fl_c = -1.8           
            fl_h = 0.4 * math.sin(self.wave_step)

        point = JointTrajectoryPoint()
        point.positions = [fl_h, fl_t, fl_c, fr_h, fr_t, fr_c, rl_h, rl_t, rl_c, rr_h, rr_t, rr_c]
        point.time_from_start.nanosec = 50000000 
        traj.points.append(point)
        self.pub.publish(traj)

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = JaxTeleopBody()
    
    try:
        while rclpy.ok():
            key = node.get_key(settings)
            # We now update TARGETS instead of the raw variables
            if key == 'w': node.t_pitch += 0.05
            elif key == 's': node.t_pitch -= 0.05
            elif key == 'a': node.t_roll -= 0.05
            elif key == 'd': node.t_roll += 0.05
            elif key == 'r': node.t_height -= 0.05
            elif key == 'f': node.t_height += 0.05
            elif key == 'h': 
                node.is_waving = not node.is_waving
                if not node.is_waving: node.wave_step = 0.0
            elif key == 'k': node.is_sitting = not node.is_sitting
            elif key == ' ': 
                # Spacebar just zeroes the targets—the LERP handles the rest!
                node.t_pitch, node.t_roll, node.t_height = 0.0, 0.0, 0.0
                node.is_waving = False
                node.is_sitting = False
            elif key == '\x03': break
            
            rclpy.spin_once(node, timeout_sec=0.01)
            
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()