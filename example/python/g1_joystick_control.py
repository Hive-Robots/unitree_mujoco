import time
import numpy as np
import pygame

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC

# Constants
dt = 0.002  # 500Hz control loop
crc = CRC()
MAX_KNEE_ANGLE = np.deg2rad(100)  # Limit to ±100 deg = ±1.74 rad

# Knee joint indices
L_KNEE = 3
R_KNEE = 9

def init_joystick():
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() < 1:
        raise RuntimeError("No joystick detected")
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print("✅ Joystick initialized:", joystick.get_name())
    return joystick

def get_axis(joystick, index):
    return np.clip(joystick.get_axis(index), -1.0, 1.0)

def init_lowcmd():
    cmd = unitree_go_msg_dds__LowCmd_()
    cmd.head[0] = 0xFE
    cmd.head[1] = 0xEF
    cmd.level_flag = 0xFF
    cmd.gpio = 0
    for i in range(20):
        cmd.motor_cmd[i].mode = 0x01
        cmd.motor_cmd[i].kp = 30.0
        cmd.motor_cmd[i].kd = 3.5
        cmd.motor_cmd[i].dq = 0.0
        cmd.motor_cmd[i].tau = 0.0
        cmd.motor_cmd[i].q = 0.0
    return cmd

if __name__ == '__main__':
    input("🔧 Press Enter to start controlling knees...")

    print("🔌 Initializing SDK channel...")
    ChannelFactoryInitialize(1, "lo")
    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()
    print("✅ Ready to send commands on 'rt/lowcmd'")

    joystick = init_joystick()
    cmd = init_lowcmd()

    frame = 0
    while True:
        step_start = time.perf_counter()
        pygame.event.pump()

        # Read joystick vertical axes
        ly = get_axis(joystick, 1)  # Left stick Y → Left knee
        ry = get_axis(joystick, 4)  # Right stick Y → Right knee

        # Compute knee joint positions
        l_knee_q = ly * MAX_KNEE_ANGLE
        r_knee_q = ry * MAX_KNEE_ANGLE

        # Apply positions only to knee joints
        cmd.motor_cmd[L_KNEE].q = l_knee_q
        cmd.motor_cmd[R_KNEE].q = r_knee_q

        cmd.crc = crc.Crc(cmd)
        pub.Write(cmd)

        # Print every 20 frames (~10 Hz)
        if frame % 20 == 0:
            print(f"\n🎮 Frame {frame}")
            print(f"  L_KNEE (joint 3): {l_knee_q:+.3f} rad")
            print(f"  R_KNEE (joint 9): {r_knee_q:+.3f} rad")
            print("📤 LowCmd sent with knee positions.")

        frame += 1
        elapsed = time.perf_counter() - step_start
        time.sleep(max(0.0, dt - elapsed))
