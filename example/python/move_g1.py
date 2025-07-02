import math
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC


def main():
    ChannelFactoryInitialize(1, "lo")
    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()

    cmd = unitree_hg_msg_dds__LowCmd_()
    cmd.head[0] = 0xFE
    cmd.head[1] = 0xEF
    cmd.level_flag = 0xFF
    cmd.gpio = 0

    num_motors = 29  # g1 29dof without hands
    for i in range(num_motors):
        cmd.motor_cmd[i].mode = 0x01  # PMSM mode
        cmd.motor_cmd[i].q = 0.0
        cmd.motor_cmd[i].kp = 30.0
        cmd.motor_cmd[i].dq = 0.0
        cmd.motor_cmd[i].kd = 2.0
        cmd.motor_cmd[i].tau = 0.0

    crc = CRC()
    t = 0.0
    dt = 0.002
    freq = 0.5  # Hz
    amplitude = 0.5  # rad

    while True:
        angle = amplitude * math.sin(2 * math.pi * freq * t)
        for i in range(num_motors):
            cmd.motor_cmd[i].q = angle
        cmd.crc = crc.Crc(cmd)
        pub.Write(cmd)
        t += dt
        time.sleep(dt)


if __name__ == "__main__":
    main()