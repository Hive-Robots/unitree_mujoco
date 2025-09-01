#!/usr/bin/env python3
import time, sys, math
import numpy as np

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_ as LowCmdDefault
from unitree_sdk2py.utils.crc import CRC

# -------------------------- Timing --------------------------
HZ = 500.0
DT = 1.0 / HZ

# -------------------------- Joint indices (G1) --------------------------
class J:
    LeftHipPitch=0; LeftHipRoll=1; LeftHipYaw=2; LeftKnee=3
    LeftAnkPitch=4; LeftAnkB=4; LeftAnkRoll=5; LeftAnkA=5
    RightHipPitch=6; RightHipRoll=7; RightHipYaw=8; RightKnee=9
    RightAnkPitch=10; RightAnkB=10; RightAnkRoll=11; RightAnkA=11
    WaistYaw=12; WaistRoll=13; WaistA=13; WaistPitch=14; WaistB=14
    LShPitch=15; LShRoll=16; LShYaw=17; LElb=18
    LWristRoll=19; LWristPitch=20; LWristYaw=21
    RShPitch=22; RShRoll=23; RShYaw=24; RElb=25
    RWristRoll=26; RWristPitch=27; RWristYaw=28

N = 29

# -------------------------- Gains (like the example) --------------------------
KP = np.array([
    60,60,60,100,40,40,  60,60,60,100,40,40,  60,40,40,
    40,40,40,40,40,40,40,  40,40,40,40,40,40,40
], dtype=float)
KD = np.array([
    1,1,1,2,1,1,  1,1,1,2,1,1,  1,1,1,
    1,1,1,1,1,1,1,  1,1,1,1,1,1,1
], dtype=float)

# -------------------------- Walking motion (fixed) --------------------------
π = math.pi

# Same gait spec as your ROS2 node (oscillators) — no velocity inputs
GAIT_CFG = {
    'left_hip_pitch_joint':   {'amp': 0.45, 'off':  0.00, 'phi': 0.0},
    'right_hip_pitch_joint':  {'amp': 0.45, 'off':  0.00, 'phi': π},
    'left_knee_joint':        {'amp': 0.35, 'off':  0.50, 'phi': 0.4},
    'right_knee_joint':       {'amp': 0.35, 'off':  0.50, 'phi': 0.4 + π},
    'left_ankle_pitch_joint': {'amp': 0.50, 'off': -0.10, 'phi': -0.4},
    'right_ankle_pitch_joint':{'amp': 0.50, 'off': -0.10, 'phi': -0.4 + π},
    'left_hip_roll_joint':    {'amp': 0.01, 'off':  0.00, 'phi': 0.5*π},
    'right_hip_roll_joint':   {'amp': 0.01, 'off':  0.00, 'phi': 0.5*π + π},
    'left_ankle_roll_joint':  {'amp': 0.06, 'off':  0.00, 'phi': -0.5*π},
    'right_ankle_roll_joint': {'amp': 0.06, 'off':  0.00, 'phi': -0.5*π + π},
    'left_hip_yaw_joint':     {'amp': 0.01, 'off':  0.00, 'phi': 0.0},
    'right_hip_yaw_joint':    {'amp': 0.01, 'off':  0.00, 'phi': π},
    'left_shoulder_pitch_joint':  {'amp': 0.15, 'off': 0.00, 'phi': π},
    'right_shoulder_pitch_joint': {'amp': 0.15, 'off': 0.00, 'phi': 0.0},
}

# Static joints (constant angles)
STATIC_CFG = {
    'left_elbow_joint':  1.2,
    'right_elbow_joint': 1.2,
    'left_shoulder_roll_joint':  0.134,
    'right_shoulder_roll_joint': -0.134,
}

# ROS name -> index mapping
NAME2IDX = {
    'left_hip_pitch_joint': J.LeftHipPitch,
    'right_hip_pitch_joint': J.RightHipPitch,
    'left_knee_joint': J.LeftKnee,
    'right_knee_joint': J.RightKnee,
    'left_ankle_pitch_joint': J.LeftAnkPitch,
    'right_ankle_pitch_joint': J.RightAnkPitch,
    'left_hip_roll_joint': J.LeftHipRoll,
    'right_hip_roll_joint': J.RightHipRoll,
    'left_ankle_roll_joint': J.LeftAnkRoll,
    'right_ankle_roll_joint': J.RightAnkRoll,
    'left_hip_yaw_joint': J.LeftHipYaw,
    'right_hip_yaw_joint': J.RightHipYaw,
    'left_shoulder_pitch_joint': J.LShPitch,
    'right_shoulder_pitch_joint': J.RShPitch,
    'left_shoulder_roll_joint': J.LShRoll,
    'right_shoulder_roll_joint': J.RShRoll,
    'left_elbow_joint': J.LElb,
    'right_elbow_joint': J.RElb,
}

# Fixed cadence and amplitude scale (no cmd_vel)
FREQ_HZ = 0.5            # walking cadence
AMP_SCALE = 1.0          # overall amplitude multiplier

# Optional: capture state (not required for motion)
def _state_cb(_msg: LowState_): 
    pass

if __name__ == '__main__':
    print("Simulation mode: fixed walking motion (no velocity inputs).")
    input("Press Enter to start...")

    # Loopback so your MuJoCo bridge sees the traffic, like your working script. :contentReference[oaicite:2]{index=2}
    if len(sys.argv) < 2:
        ChannelFactoryInitialize(1, "lo")
    else:
        ChannelFactoryInitialize(0, sys.argv[1])

    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(_state_cb)

    cmd = LowCmdDefault()
    crc = CRC()

    # Enable all joints once
    for i in range(N):
        cmd.motor_cmd[i].mode = 0x0A

    theta = 0.0
    last = time.perf_counter()

    while True:
        tic = time.perf_counter()
        dt = tic - last
        last = tic
        if dt <= 0.0 or dt > 0.1:
            dt = DT

        # integrate shared phase
        theta += 2.0 * math.pi * FREQ_HZ * dt
        if theta > 2.0 * math.pi:
            theta = math.fmod(theta, 2.0 * math.pi)

        # desired positions
        qdes = np.zeros(N, dtype=float)

        # oscillators
        for name, cfg in GAIT_CFG.items():
            idx = NAME2IDX[name]
            amp = cfg['amp'] * AMP_SCALE
            qdes[idx] = cfg['off'] + amp * math.sin(theta + cfg['phi'])

        # statics
        qdes[NAME2IDX['left_elbow_joint']]  = STATIC_CFG['left_elbow_joint']
        qdes[NAME2IDX['right_elbow_joint']] = STATIC_CFG['right_elbow_joint']
        qdes[NAME2IDX['left_shoulder_roll_joint']]  = STATIC_CFG['left_shoulder_roll_joint']
        qdes[NAME2IDX['right_shoulder_roll_joint']] = STATIC_CFG['right_shoulder_roll_joint']

        # fill and publish full vector
        for i in range(N):
            cmd.motor_cmd[i].q  = float(qdes[i])
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].kp = float(KP[i])
            cmd.motor_cmd[i].kd = float(KD[i])
            cmd.motor_cmd[i].tau = 0.0

        cmd.crc = crc.Crc(cmd)
        pub.Write(cmd)

        # pace loop
        sleep_t = DT - (time.perf_counter() - tic)
        if sleep_t > 0:
            time.sleep(sleep_t)
