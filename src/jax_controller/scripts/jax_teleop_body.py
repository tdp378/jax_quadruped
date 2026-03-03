#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys, select, termios, tty
import math

msg = """
JAX IK - Full Control (Fixed Wave & Sit)
---------------------------
W / S : Lean Forward / Back
A / D : Lean Left / Right
R / F : Change Height
H     : TOGGLE WAVE
K     : TOGGLE SIT
T     : TEST STEP (Front Left)
Space : Reset All
CTRL-C to quit
"""

class JaxTeleopBody(Node):
    def __init__(self):
        super().__init__('jax_teleop_body')
        self.pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        
        # --- Physical Constants ---
        self.L1 = 0.130011  
        self.L2 = 0.132389  
        self.perfect_h = 0.18  
        
        # State Variables
        self.pitch, self.roll, self.height = 0.0, 0.0, 0.18
        self.t_pitch, self.t_roll, self.t_height = 0.0, 0.0, 0.18
        
        self.is_waving = False
        self.wave_step = 0.0
        self.is_sitting = False
        self.sit_lerp = 0.0
        self.is_test_stepping = False
        self.step_phase = 0.0 
        
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
        thigh_int = math.acos(max(-1.0, min(1.0, cos_t_targ)))
        abs_thigh_targ = (thigh_int - math.atan2(x, z))
        
        # Reference (0.18m)
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
        
        # 1. SMOOTHING
        self.pitch += (self.t_pitch - self.pitch) * 0.15
        self.roll += (self.t_roll - self.roll) * 0.15
        self.height += (self.t_height - self.height) * 0.15

        # Smooth Sit Transition
        if self.is_sitting and self.sit_lerp < 1.0: self.sit_lerp += 0.05
        elif not self.is_sitting and self.sit_lerp > 0.0: self.sit_lerp -= 0.05

        # 2. COORDINATE PREP
        # Normal Body Height Logic
        z_fl = self.height - (self.pitch * 0.1) + (self.roll * 0.1)
        z_fr = self.height - (self.pitch * 0.1) - (self.roll * 0.1)
        z_rl = self.height + (self.pitch * 0.1) + (self.roll * 0.1)
        z_rr = self.height + (self.pitch * 0.1) - (self.roll * 0.1)
        
        # Sitting Logic (Front Legs stay at 0.23m, Rear Legs lerp down to sit height)
        z_fl = z_fl * (1 - self.sit_lerp) + 0.23 * self.sit_lerp
        z_fr = z_fr * (1 - self.sit_lerp) + 0.23 * self.sit_lerp
        # Apply SIT LERP (Front stays up, Rear goes down)
        sit_height = 0.10 
        z_rl = z_rl * (1 - self.sit_lerp) + sit_height * self.sit_lerp
        z_rr = z_rr * (1 - self.sit_lerp) + sit_height * self.sit_lerp

        # 3. TEST STEP OVERRIDE (Front Left)
        step_x, step_z_off = 0.0, 0.0
        if self.is_test_stepping:
            self.step_phase += 0.1
            if self.step_phase > 1.0: 
                self.step_phase = 0.0
                self.is_test_stepping = False
            step_z_off = -0.04 * math.sin(math.pi * self.step_phase)
            step_x = -0.05 * math.sin(math.pi * self.step_phase)
            z_fl += step_z_off

        # 4. SOLVE IK
        fl_t, fl_c = self.solve_ik(step_x, z_fl)
        fr_t, fr_c = self.solve_ik(0, z_fr)
        rl_t, rl_c = self.solve_ik(0, z_rl)
        rr_t, rr_c = self.solve_ik(0, z_rr)
        
        fl_h = fr_h = rl_h = rr_h = 0.0

        # 5. WAVE OVERRIDE (Total Manual Control for Wave)
        if self.is_waving:
            self.wave_step += 0.3
            # Lift FL high and tuck calf
            fl_t = -1.0
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
            if key == 'w': node.t_pitch += 0.05
            elif key == 's': node.t_pitch -= 0.05
            elif key == 'a': node.t_roll -= 0.05
            elif key == 'd': node.t_roll += 0.05
            elif key == 'r': node.t_height = min(0.24, node.t_height + 0.01)
            elif key == 'f': node.t_height = max(0.08, node.t_height - 0.01)
            elif key == 'h': 
                node.is_waving = not node.is_waving
                if not node.is_waving: node.wave_step = 0.0
            elif key == 'k': node.is_sitting = not node.is_sitting
            elif key == 't': 
                node.is_test_stepping = True
                node.step_phase = 0.0
            elif key == ' ': 
                node.t_pitch, node.t_roll, node.t_height = 0.0, 0.0, 0.18
                node.is_sitting = False
                node.is_waving = False
            elif key == '\x03': break
            rclpy.spin_once(node, timeout_sec=0.01)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()