import argparse
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_


# Per-side absolute joint targets, indexed by Dex3_1 motor index per g1_joint_index_dds.md:
# 0 thumb_0, 1 thumb_1, 2 thumb_2, 3 middle_0, 4 middle_1, 5 index_0, 6 index_1.
# Right-hand signs flip because several joint ranges mirror the left side.
POSES = {
    "open":  {"left":  [0.0] * 7,
              "right": [0.0] * 7},
    "fist":  {"left":  [0.0,  0.7,  1.7, -1.5, -1.7, -1.5, -1.7],
              "right": [0.0, -0.7, -1.7,  1.5,  1.7,  1.5,  1.7]},
    "pinch": {"left":  [-0.5,  0.5,  0.0, -1.5, -1.7,  -1.4, -0.3],
              "right": [-0.5,  -0.5,  0.0, 1.5,  1.7,  1.4,  0.3]},
    "thumbs_up": {"left":  [0.0,  0.0,  0.0, -1.5, -1.7, -1.5, -1.7],
                  "right": [0.0,  0.0,  0.0,  1.5,  1.7,  1.5,  1.7]},
}

KP, KD = 1.5, 0.2


def fill(msg, targets):
    for i, q in enumerate(targets):
        m = msg.motor_cmd[i]
        m.q = q
        m.dq = 0.0
        m.tau = 0.0
        m.kp = KP
        m.kd = KD


def main():
    p = argparse.ArgumentParser(description="Publish a named Dex3 hand pose to the simulator.")
    p.add_argument("pose", choices=list(POSES))
    p.add_argument("--domain", type=int, default=0, help="DDS domain id (sim default: 0)")
    p.add_argument("--iface", default="lo", help="Network interface (sim default: lo)")
    p.add_argument("--hold", type=float, default=2.0, help="Seconds to keep publishing")
    a = p.parse_args()

    ChannelFactoryInitialize(a.domain, a.iface)

    pubs = {
        "left":  ChannelPublisher("rt/dex3/left/cmd",  HandCmd_),
        "right": ChannelPublisher("rt/dex3/right/cmd", HandCmd_),
    }
    msgs = {side: unitree_hg_msg_dds__HandCmd_() for side in pubs}
    for side, pub in pubs.items():
        pub.Init()
        fill(msgs[side], POSES[a.pose][side])

    t_end = time.time() + a.hold
    while time.time() < t_end:
        for side, pub in pubs.items():
            pub.Write(msgs[side])
        time.sleep(0.02)


if __name__ == "__main__":
    main()
