import time
import numpy as np
import sys
import threading

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_ as LowCmdDefault
from unitree_sdk2py.utils.crc import CRC

# Define joint positions
stand_up_joint_pos = np.zeros(29)
stand_down_joint_pos = np.array([
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    -2.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
])

# Global state variable
latest_joint_positions = np.zeros(29)
dt = 0.002  # 500 Hz
crc = CRC()
last_print_time = 0.0  # for throttling output
print_interval = 0.5   # seconds

def state_callback(state: LowState_):
    global latest_joint_positions, last_print_time
    now = time.time()
    
    for i in range(29):
        latest_joint_positions[i] = state.motor_state[i].q

    # Only print if interval has passed
    if now - last_print_time >= print_interval:
        print("Current joint positions:\n", np.round(latest_joint_positions, 3))
        last_print_time = now


if __name__ == '__main__':
    input("Press Enter to start...")

    if len(sys.argv) < 2:
        ChannelFactoryInitialize(1, "lo")
    else:
        ChannelFactoryInitialize(0, sys.argv[1])

    # Publisher and Subscriber setup
    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()

    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(state_callback)

    # Prepare command
    cmd = LowCmdDefault()
    for i in range(29):
        cmd.motor_cmd[i].mode = 0x0A

    running_time = 0.0
    cycle_duration = 8.0

    while True:
        step_start = time.perf_counter()
        t_mod = running_time % cycle_duration
        half_cycle = cycle_duration / 2.0

        if t_mod < half_cycle:
            phase = 0.5 * (1 - np.cos(np.pi * t_mod / half_cycle))
            from_pose = stand_down_joint_pos
            to_pose = stand_up_joint_pos
        else:
            phase = 0.5 * (1 - np.cos(np.pi * (t_mod - half_cycle) / half_cycle))
            from_pose = stand_up_joint_pos
            to_pose = stand_down_joint_pos

        for i in range(29):
            cmd.motor_cmd[i].q = (1 - phase) * from_pose[i] + phase * to_pose[i]
            cmd.motor_cmd[i].kp = 5.0
            cmd.motor_cmd[i].kd = 0.2
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].tau = 0.0

        cmd.crc = crc.Crc(cmd)
        pub.Write(cmd)

        running_time += dt
        time_to_sleep = dt - (time.perf_counter() - step_start)
        if time_to_sleep > 0:
            time.sleep(time_to_sleep)
