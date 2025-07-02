import time
import numpy as np
import sys

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_              # G1-specific LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_ as LowCmdDefault
from unitree_sdk2py.utils.crc import CRC

# Standing and squatting joint positions for G1
stand_up_joint_pos = np.array([
    0.0057, 0.61, -1.22,     # FL
   -0.0057, 0.61, -1.22,     # FR
    0.0057, 0.61, -1.22,     # RL
   -0.0057, 0.61, -1.22      # RR
])

stand_down_joint_pos = np.array([
    0.0473, 1.22, -2.44,
   -0.0473, 1.22, -2.44,
    0.0473, 1.22, -2.44,
   -0.0473, 1.22, -2.44
])

# Simulation parameters
dt = 0.002  # 500 Hz
running_time = 0.0
crc = CRC()

input("Press Enter to start...")

if __name__ == '__main__':

    # Initialize DDS with loopback
    if len(sys.argv) < 2:
        ChannelFactoryInitialize(1, "lo")  # Use loopback interface
    else:
        ChannelFactoryInitialize(0, sys.argv[1])  # Custom interface

    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()

    # Create and configure LowCmd
    cmd = LowCmdDefault()
    for i in range(12):  # G1 has 12 joints
        cmd.motor_cmd[i].mode = 0x0A  # Position + PD + feedforward torque

    while True:
        step_start = time.perf_counter()
        running_time += dt

        if running_time < 3.0:
            # Stand up phase
            phase = np.tanh(running_time / 1.2)
            for i in range(12):
                cmd.motor_cmd[i].q = phase * stand_up_joint_pos[i] + (1 - phase) * stand_down_joint_pos[i]
                cmd.motor_cmd[i].kp = 20.0 + 30.0 * phase
                cmd.motor_cmd[i].kd = 3.5
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].tau = 0.0

        else:
            # Squat down phase
            phase = np.tanh((running_time - 3.0) / 1.2)
            for i in range(12):
                cmd.motor_cmd[i].q = phase * stand_down_joint_pos[i] + (1 - phase) * stand_up_joint_pos[i]
                cmd.motor_cmd[i].kp = 50.0
                cmd.motor_cmd[i].kd = 3.5
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].tau = 0.0

        # Compute and set CRC
        cmd.crc = crc.Crc(cmd)

        # Send command
        pub.Write(cmd)

        # Real-time rate control
        time_to_sleep = dt - (time.perf_counter() - step_start)
        if time_to_sleep > 0:
            time.sleep(time_to_sleep)
