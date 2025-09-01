import time
import sys
import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_ as LowCmdDefault
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread

G1_NUM_MOTOR = 29

# Gains copied from the original example
Kp = [
    60, 60, 60, 100, 40, 40,      # legs
    60, 60, 60, 100, 40, 40,      # legs
    60, 40, 40,                   # waist
    40, 40, 40, 40,  40, 40, 40,  # arms
    40, 40, 40, 40,  40, 40, 40   # arms
]
Kd = [
    1, 1, 1, 2, 1, 1,     # legs
    1, 1, 1, 2, 1, 1,     # legs
    1, 1, 1,              # waist
    1, 1, 1, 1, 1, 1, 1,  # arms
    1, 1, 1, 1, 1, 1, 1   # arms
]

class G1JointIndex:
    LeftHipPitch = 0; LeftHipRoll = 1; LeftHipYaw = 2; LeftKnee = 3
    LeftAnklePitch = 4; LeftAnkleB = 4; LeftAnkleRoll = 5; LeftAnkleA = 5
    RightHipPitch = 6; RightHipRoll = 7; RightHipYaw = 8; RightKnee = 9
    RightAnklePitch = 10; RightAnkleB = 10; RightAnkleRoll = 11; RightAnkleA = 11
    WaistYaw = 12; WaistRoll = 13; WaistA = 13; WaistPitch = 14; WaistB = 14
    LeftShoulderPitch = 15; LeftShoulderRoll = 16; LeftShoulderYaw = 17; LeftElbow = 18
    LeftWristRoll = 19; LeftWristPitch = 20; LeftWristYaw = 21
    RightShoulderPitch = 22; RightShoulderRoll = 23; RightShoulderYaw = 24; RightElbow = 25
    RightWristRoll = 26; RightWristPitch = 27; RightWristYaw = 28

class Mode:
    PR = 0
    AB = 1

class Custom:
    def __init__(self):
        self.time_ = 0.0
        self.control_dt_ = 0.002  # 2 ms
        self.duration_ = 3.0
        self.low_cmd = LowCmdDefault()
        self.low_state = None
        self.mode_machine_ = 0  # not used by sim bridge, but we keep the field
        self.crc = CRC()
        self._print_counter = 0

    def Init(self):
        # No MotionSwitcher handshake in simulation.
        # Create publisher/subscriber directly.
        self.lowcmd_publisher_ = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher_.Init()

        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

        # Enable all motors once up front (0x0A is the common enable bitmask)
        for i in range(G1_NUM_MOTOR):
            self.low_cmd.motor_cmd[i].mode = 0x0A

        # Start control immediately; do not block on first state
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=self.control_dt_, target=self.LowCmdWrite, name="control"
        )
        self.lowCmdWriteThreadPtr.Start()

    def LowStateHandler(self, msg: LowState_):
        self.low_state = msg
        self._print_counter += 1
        if self._print_counter % 500 == 0:
            self._print_counter = 0
            # Throttled IMU print so you can see sim is alive
            try:
                print("IMU rpy:", [round(v, 3) for v in msg.imu_state.rpy])
            except Exception:
                pass

    def _safe_q(self, i):
        # If we haven't received state yet, assume zero angle
        if self.low_state is None:
            return 0.0
        return float(self.low_state.motor_state[i].q)

    def LowCmdWrite(self):
        self.time_ += self.control_dt_

        # Always set shared fields for all joints each tick
        for i in range(G1_NUM_MOTOR):
            self.low_cmd.motor_cmd[i].kp = float(Kp[i])
            self.low_cmd.motor_cmd[i].kd = float(Kd[i])
            self.low_cmd.motor_cmd[i].tau = 0.0  # let your bridge form PD torques

        if self.time_ < self.duration_:
            # Stage 1: smoothly move to zero posture (from current, or from zero if no state yet)
            ratio = np.clip(self.time_ / self.duration_, 0.0, 1.0)
            for i in range(G1_NUM_MOTOR):
                q_now = self._safe_q(i)
                self.low_cmd.motor_cmd[i].q = (1.0 - ratio) * q_now
                self.low_cmd.motor_cmd[i].dq = 0.0

        elif self.time_ < self.duration_ * 2:
            # Stage 2: PR ankle swing
            max_P = np.pi * 30.0 / 180.0
            max_R = np.pi * 10.0 / 180.0
            t = self.time_ - self.duration_
            L_P_des = max_P * np.sin(2.0 * np.pi * t)
            L_R_des = max_R * np.sin(2.0 * np.pi * t)
            R_P_des = max_P * np.sin(2.0 * np.pi * t)
            R_R_des = -max_R * np.sin(2.0 * np.pi * t)

            # Keep everything else near zero with damping, only drive ankles
            for i in range(G1_NUM_MOTOR):
                self.low_cmd.motor_cmd[i].q = 0.0
                self.low_cmd.motor_cmd[i].dq = 0.0

            self.low_cmd.motor_cmd[G1JointIndex.LeftAnklePitch].q = L_P_des
            self.low_cmd.motor_cmd[G1JointIndex.LeftAnkleRoll].q  = L_R_des
            self.low_cmd.motor_cmd[G1JointIndex.RightAnklePitch].q = R_P_des
            self.low_cmd.motor_cmd[G1JointIndex.RightAnkleRoll].q  = R_R_des

        else:
            # Stage 3: AB ankle swing (+ gentle wrist roll)
            max_A = np.pi * 30.0 / 180.0
            max_B = np.pi * 10.0 / 180.0
            t = self.time_ - self.duration_ * 2
            L_A_des = max_A * np.sin(2.0 * np.pi * t)
            L_B_des = max_B * np.sin(2.0 * np.pi * t + np.pi)
            R_A_des = -max_A * np.sin(2.0 * np.pi * t)
            R_B_des = -max_B * np.sin(2.0 * np.pi * t + np.pi)

            for i in range(G1_NUM_MOTOR):
                self.low_cmd.motor_cmd[i].q = 0.0
                self.low_cmd.motor_cmd[i].dq = 0.0

            # Map AB into indices as in the original example
            self.low_cmd.motor_cmd[G1JointIndex.LeftAnkleA].q  = L_A_des
            self.low_cmd.motor_cmd[G1JointIndex.LeftAnkleB].q  = L_B_des
            self.low_cmd.motor_cmd[G1JointIndex.RightAnkleA].q = R_A_des
            self.low_cmd.motor_cmd[G1JointIndex.RightAnkleB].q = R_B_des

            max_WristYaw = np.pi * 30.0 / 180.0
            L_WristYaw_des = max_WristYaw * np.sin(2.0 * np.pi * t)
            R_WristYaw_des = max_WristYaw * np.sin(2.0 * np.pi * t)
            self.low_cmd.motor_cmd[G1JointIndex.LeftWristRoll].q  = L_WristYaw_des
            self.low_cmd.motor_cmd[G1JointIndex.RightWristRoll].q = R_WristYaw_des

        # Publish
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher_.Write(self.low_cmd)

if __name__ == '__main__':
    print("WARNING: Simulation mode. Ensure your MuJoCo bridge is running.")
    input("Press Enter to continue...")

    # Default to loopback like your working MuJoCo-friendly script; allow override with IP
    if len(sys.argv) < 2:
        ChannelFactoryInitialize(1, "lo")
    else:
        ChannelFactoryInitialize(0, sys.argv[1])

    custom = Custom()
    custom.Init()

    while True:
        time.sleep(1)
