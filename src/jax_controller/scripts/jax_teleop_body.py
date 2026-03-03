#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys, select, termios, tty
import math

msg = """
JAX IK - GIMBAL MASTER (Symmetry Fix)
---------------------------
W / S : Pitch Forward / Back
A / D : Roll Left / Right
I / K : SLIDE Body Forward / Back
J / L : SLIDE Body Left / Right
R / F : Change Height
N     : TOGGLE SIT
M     : TOGGLE WAVE
T     : TEST STEP
Space : Reset All
CTRL-C to quit
"""

class JaxTeleopBody(Node):
    def __init__(self):
        super().__init__('jax_teleop_body')
        self.pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        
        self.L1, self.L2 = 0.130011, 0.132389
        self.perfect_h = 0.18
        
        self.pitch, self.roll, self.height = 0.0, 0.0, 0.18
        self.t_pitch, self.t_roll, self.t_height = 0.0, 0.0, 0.18
        
        self.body_x, self.body_y = 0.0, 0.0
        self.t_body_x, self.t_body_y = 0.0, 0.0
        
        self.is_waving, self.wave_step = False, 0.0
        self.is_sitting, self.sit_lerp = False, 0.0
        self.is_test_stepping, self.step_phase = False, 0.0 
        
        self.joint_names = [
            'fl_hip_joint', 'fl_thigh_joint', 'fl_calf_joint',
            'fr_hip_joint', 'fr_thigh_joint', 'fr_calf_joint',
            'rl_hip_joint', 'rl_thigh_joint', 'rl_calf_joint',
            'rr_hip_joint', 'rr_thigh_joint', 'rr_calf_joint'
        ]
        
        self.timer = self.create_timer(0.05, self.publish_stance)
        print(msg)

    def solve_ik(self, x, z):
        l1, l2 = self.L1, self.L2
        z = max(0.05, min(l1 + l2 - 0.01, abs(z)))
        d_target = math.sqrt(x**2 + z**2)
        d_target = max(0.05, min(l1 + l2 - 0.001, d_target))
        
        cos_c_targ = (l1**2 + l2**2 - d_target**2) / (2 * l1 * l2)
        abs_calf_targ = -(math.pi - math.acos(max(-1.0, min(1.0, cos_c_targ))))
        cos_t_targ = (l1**2 + d_target**2 - l2**2) / (2 * l1 * d_target)
        abs_thigh_targ = (math.acos(max(-1.0, min(1.0, cos_t_targ))) - math.atan2(x, z))
        
        d_perf = self.perfect_h
        cos_c_perf = (l1**2 + l2**2 - d_perf**2) / (2 * l1 * l2)
        abs_calf_perf = -(math.pi - math.acos(max(-1.0, min(1.0, cos_c_perf))))
        cos_t_perf = (l1**2 + d_perf**2 - l2**2) / (2 * l1 * d_perf)
        abs_thigh_perf = math.acos(max(-1.0, min(1.0, cos_t_perf)))

        return (abs_thigh_targ - abs_thigh_perf), (abs_calf_targ - abs_calf_perf)

    def get_key(self, settings):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1).lower() if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def publish_stance(self):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        
        # Smooth Transitions
        self.pitch += (self.t_pitch - self.pitch) * 0.15
        self.roll += (self.t_roll - self.roll) * 0.15
        self.height += (self.t_height - self.height) * 0.15
        self.body_x += (self.t_body_x - self.body_x) * 0.15
        self.body_y += (self.t_body_y - self.body_y) * 0.15

        if self.is_sitting and self.sit_lerp < 1.0: self.sit_lerp += 0.05
        elif not self.is_sitting and self.sit_lerp > 0.0: self.sit_lerp -= 0.05

        # 1. Coordinate Prep
        z_fl = self.height - (self.pitch * 0.1) + (self.roll * 0.1)
        z_fr = self.height - (self.pitch * 0.1) - (self.roll * 0.1)
        z_rl = self.height + (self.pitch * 0.1) + (self.roll * 0.1)
        z_rr = self.height + (self.pitch * 0.1) - (self.roll * 0.1)
        
        # 2. Sit Logic
        z_fl = z_fl * (1 - self.sit_lerp) + 0.23 * self.sit_lerp
        z_fr = z_fr * (1 - self.sit_lerp) + 0.23 * self.sit_lerp
        z_rl = z_rl * (1 - self.sit_lerp) + 0.10 * self.sit_lerp
        z_rr = z_rr * (1 - self.sit_lerp) + 0.10 * self.sit_lerp

        # 3. Hip/Gimbal Y-Logic (SYMMETRY FIX)
        hip_angle = math.atan2(self.body_y, self.height)
        fl_h = fr_h = hip_angle
        rl_h = rr_h = -hip_angle # Mirroring the rear to match front motion

        # 4. IK solving for X/Z
        step_x = 0.0
        if self.is_test_stepping:
            self.step_phase += 0.1
            if self.step_phase > 1.0: self.is_test_stepping = False
            step_x = -0.05 * math.sin(math.pi * self.step_phase)
            z_fl -= 0.04 * math.sin(math.pi * self.step_phase)

        fl_t, fl_c = self.solve_ik(step_x - self.body_x, z_fl)
        fr_t, fr_c = self.solve_ik(0 - self.body_x, z_fr)
        rl_t, rl_c = self.solve_ik(0 - self.body_x, z_rl)
        rr_t, rr_c = self.solve_ik(0 - self.body_x, z_rr)

        # 5. Wave Override
        if self.is_waving:
            self.wave_step += 0.3
            fl_t, fl_c, fl_h = -1.0, -1.8, 0.4 * math.sin(self.wave_step)

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
            if key == 'w': node.t_pitch += 0.05
            elif key == 's': node.t_pitch -= 0.05
            elif key == 'a': node.t_roll -= 0.05
            elif key == 'd': node.t_roll += 0.05
            elif key == 'i': node.t_body_x += 0.01
            elif key == 'k': node.t_body_x -= 0.01
            elif key == 'j': node.t_body_y -= 0.01
            elif key == 'l': node.t_body_y += 0.01
            elif key == 'r': node.t_height = min(0.24, node.t_height + 0.01)
            elif key == 'f': node.t_height = max(0.08, node.t_height - 0.01)
            elif key == 'm': node.is_waving = not node.is_waving
            elif key == 'n': node.is_sitting = not node.is_sitting
            elif key == 't': node.is_test_stepping, node.step_phase = True, 0.0
            elif key == ' ': 
                node.t_pitch = node.t_roll = node.t_body_x = node.t_body_y = 0.0
                node.t_height = 0.18
                node.is_sitting = node.is_waving = False
            elif key == '\x03': break
            rclpy.spin_once(node, timeout_sec=0.01)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()