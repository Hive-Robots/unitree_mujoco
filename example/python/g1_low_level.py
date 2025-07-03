import time
import numpy as np
import sys

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_ as LowCmdDefault
from unitree_sdk2py.utils.crc import CRC

# Define 29 joint positions (stand up and down), example values within limits
# For real application, replace with meaningful pose configurations within limits
stand_up_joint_pos = np.array([
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,     # L leg
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,     # R leg
    0.0, 0.0, 0.0,                   # Waist
    0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, # L arm
    0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0  # R arm
])

stand_down_joint_pos = np.array([
    -0.3, 0.2, 0.0, 0.5, -0.2, 0.0,
    -0.3, -0.2, 0.0, 0.5, -0.2, 0.0,
    0.0, 0.0, 0.0,
    0.0, -0.3, 0.0, 0.2, 0.0, 0.0, 0.0,
    0.0, 0.3, 0.0, 0.2, 0.0, 0.0, 0.0
])

dt = 0.002  # 500 Hz
crc = CRC()

input("Press Enter to start...")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        ChannelFactoryInitialize(1, "lo")
    else:
        ChannelFactoryInitialize(0, sys.argv[1])

    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()

    cmd = LowCmdDefault()
    for i in range(29):
        cmd.motor_cmd[i].mode = 0x0A  # Position control with PD and torque feedforward

    running_time = 0.0
    cycle_duration = 6.0  # total cycle duration (3s up, 3s down)

    while True:
        step_start = time.perf_counter()
        t_mod = running_time % cycle_duration

        if t_mod < 3.0:
            # Stand up phase
            phase = np.tanh(t_mod / 3.0)
            from_pose = stand_down_joint_pos
            to_pose = stand_up_joint_pos
        else:
            # Squat down phase
            phase = np.tanh((t_mod - 3.0) / 3.0)
            from_pose = stand_up_joint_pos
            to_pose = stand_down_joint_pos

        for i in range(29):
            cmd.motor_cmd[i].q = (1 - phase) * from_pose[i] + phase * to_pose[i]
            cmd.motor_cmd[i].kp = 30.0
            cmd.motor_cmd[i].kd = 3.5
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].tau = 0.0

        cmd.crc = crc.Crc(cmd)
        pub.Write(cmd)

        running_time += dt
        time_to_sleep = dt - (time.perf_counter() - step_start)
        if time_to_sleep > 0:
            time.sleep(time_to_sleep)
