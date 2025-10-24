import mujoco
import numpy as np
import pygame
import sys
import struct

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelPublisher

from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import WirelessController_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__WirelessController_
from unitree_sdk2py.utils.thread import RecurrentThread

import config
if config.ROBOT=="g1":
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    # Dex hand command (7 joints) for G1/H1
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandState_
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandState_ as HandState_default
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_ as LowState_default
else:
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
    from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_ as LowState_default

TOPIC_LOWCMD = "rt/lowcmd"
TOPIC_LOWSTATE = "rt/lowstate"
TOPIC_HIGHSTATE = "rt/sportmodestate"
TOPIC_WIRELESS_CONTROLLER = "rt/wirelesscontroller"

# Likely Dex3 / hand command topics (left/right). We'll subscribe to all of these:
DEX_LEFT_TOPICS = [
    "rt/dex3/left/cmd",
    "rt/g1_dex3_left_cmd",
    "rt/g1/dex3/left/cmd",
    "rt/dex3/left",  # some examples omit /cmd
]
DEX_RIGHT_TOPICS = [
    "rt/dex3/right/cmd",
    "rt/g1_dex3_right_cmd",
    "rt/g1/dex3/right/cmd",
    "rt/dex3/right",
]

MOTOR_SENSOR_NUM = 3
NUM_MOTOR_IDL_GO = 20
NUM_MOTOR_IDL_HG = 35

class UnitreeSdk2Bridge:
    def _build_hand_maps(self, names):
        """Create actuator→joint maps so we can read q/qd from the right DOF."""
        maps = []
        for n in names:
            aid = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
            if aid < 0:
                maps.append(None); continue
            jnt_id = int(self.mj_model.actuator_trnid[aid, 0])
            if jnt_id < 0:
                maps.append(None); continue
            qadr = int(self.mj_model.jnt_qposadr[jnt_id])
            dadr = int(self.mj_model.jnt_dofadr[jnt_id])
            maps.append({"aid": aid, "qadr": qadr, "dadr": dadr})
        return maps
    def __init__(self, mj_model, mj_data):
        self.mj_model = mj_model
        self.mj_data = mj_data

        self.num_motor = self.mj_model.nu
        self.dim_motor_sensor = MOTOR_SENSOR_NUM * self.num_motor
        self.have_imu = False
        self.have_frame_sensor = False
        self.dt = self.mj_model.opt.timestep
        self.idl_type = (self.num_motor > NUM_MOTOR_IDL_GO) # 0: unitree_go, 1: unitree_hg
        self.num_motor_idl = NUM_MOTOR_IDL_HG if self.idl_type else NUM_MOTOR_IDL_GO
        self.joystick = None

        # Check sensor
        for i in range(self.dim_motor_sensor, self.mj_model.nsensor):
            name = mujoco.mj_id2name(
                self.mj_model, mujoco._enums.mjtObj.mjOBJ_SENSOR, i
            )
            if name == "imu_quat":
                self.have_imu_ = True
            if name == "frame_pos":
                self.have_frame_sensor_ = True

        # Unitree sdk2 message
        self.low_state = LowState_default()
        self.low_state_puber = ChannelPublisher(TOPIC_LOWSTATE, LowState_)
        self.low_state_puber.Init()
        self.lowStateThread = RecurrentThread(
            interval=self.dt, target=self.PublishLowState, name="sim_lowstate"
        )
        self.lowStateThread.Start()

        self.high_state = unitree_go_msg_dds__SportModeState_()
        self.high_state_puber = ChannelPublisher(TOPIC_HIGHSTATE, SportModeState_)
        self.high_state_puber.Init()
        self.HighStateThread = RecurrentThread(
            interval=self.dt, target=self.PublishHighState, name="sim_highstate"
        )
        self.HighStateThread.Start()

        self.wireless_controller = unitree_go_msg_dds__WirelessController_()
        self.wireless_controller_puber = ChannelPublisher(
            TOPIC_WIRELESS_CONTROLLER, WirelessController_
        )
        self.wireless_controller_puber.Init()
        self.WirelessControllerThread = RecurrentThread(
            interval=0.01,
            target=self.PublishWirelessController,
            name="sim_wireless_controller",
        )
        self.WirelessControllerThread.Start()

        self.low_cmd_suber = ChannelSubscriber(TOPIC_LOWCMD, LowCmd_)
        self.low_cmd_suber.Init(self.LowCmdHandler, 10)

        # --- Dex hand (7-DOF per side) : subscribe & map to MuJoCo ---
        # Precompute actuator IDs by name to avoid index assumptions
        self.left_hand_names = [
            "left_hand_thumb_0", "left_hand_thumb_1", "left_hand_thumb_2",
            "left_hand_middle_0", "left_hand_middle_1",
            "left_hand_index_0", "left_hand_index_1",
            
        ]
        self.right_hand_names = [
            "right_hand_thumb_0", "right_hand_thumb_1", "right_hand_thumb_2",
            "right_hand_middle_0", "right_hand_middle_1",
            "right_hand_index_0", "right_hand_index_1",
        ]

        self.left_map  = self._build_hand_maps(self.left_hand_names)
        self.right_map = self._build_hand_maps(self.right_hand_names)

        self.left_hand_ids  = [m["aid"] if m else -1 for m in self.left_map]
        self.right_hand_ids = [m["aid"] if m else -1 for m in self.right_map]

        self.hand_aids = set(aid for aid in (self.left_hand_ids + self.right_hand_ids) if aid >= 0)

        self.hand_kp = [12.0]*7
        self.hand_kd = [0.6]*7

        self.hand_state_L = HandState_default()
        self.hand_state_R = HandState_default()

        # publish to both common namespaces
        self.dex_L_state_publishers = [
            ChannelPublisher("rt/dex3/left/state",  HandState_),
            ChannelPublisher("rt/lf/dex3/left/state",  HandState_),
        ]
        self.dex_R_state_publishers = [
            ChannelPublisher("rt/dex3/right/state", HandState_),
            ChannelPublisher("rt/lf/dex3/right/state", HandState_),
        ]
        for p in self.dex_L_state_publishers + self.dex_R_state_publishers:
            p.Init()

        # start a thread that publishes q/dq/τ for 7 joints per side
        self.HandStateThread = RecurrentThread(
            interval=self.dt, target=self.PublishHandState, name="sim_handstate"
        )
        self.HandStateThread.Start()

        # Subscribe to a few possible left/right topics (any that are wrong just won't deliver)
        self.dex_left_subs = []
        self.dex_right_subs = []
        for t in DEX_LEFT_TOPICS:
            s = ChannelSubscriber(t, HandCmd_)
            s.Init(self.DexLeftHandler, 10)
            self.dex_left_subs.append(s)
        for t in DEX_RIGHT_TOPICS:
            s = ChannelSubscriber(t, HandCmd_)
            s.Init(self.DexRightHandler, 10)
            self.dex_right_subs.append(s)

        # joystick
        self.key_map = {
            "R1": 0,
            "L1": 1,
            "start": 2,
            "select": 3,
            "R2": 4,
            "L2": 5,
            "F1": 6,
            "F2": 7,
            "A": 8,
            "B": 9,
            "X": 10,
            "Y": 11,
            "up": 12,
            "right": 13,
            "down": 14,
            "left": 15,
        }


    def _apply_hand_pd(self, amap, q_des):
        if not q_des: return
        N = min(len(amap), len(q_des))
        for i in range(N):
            entry = amap[i]
            if not entry: continue
            aid  = entry["aid"]
            qadr = entry["qadr"]
            dadr = entry["dadr"]
            q  = float(self.mj_data.qpos[qadr])
            dq = float(self.mj_data.qvel[dadr])
            tau = self.hand_kp[i]*(q_des[i] - q) - self.hand_kd[i]*dq

            lo, hi = self.mj_model.actuator_ctrlrange[aid]
            if lo < hi:  # clamp to actuator ctrl range if defined
                tau = max(lo, min(hi, tau))
            self.mj_data.ctrl[aid] = tau

    def LowCmdHandler(self, msg: LowCmd_):
        """Apply LowCmd to non-hand actuators with PD+feedforward; leave hands to Dex PD."""
        try:
            if self.mj_data is None:
                return

            motor_cmd = getattr(msg, "motor_cmd", None)
            if not motor_cmd:
                return

            N_idl = len(motor_cmd)                    # how many motors the IDL carries (e.g., 35 on HG)
            N_sim = self.num_motor                    # how many actuators in the MuJoCo model
            N = min(N_idl, N_sim)

            # iterate only through actuators covered by the IDL
            for i in range(N):
                # skip hands — controlled elsewhere
                if i in self.hand_aids:
                    continue

                cmd   = motor_cmd[i]
                tauff = float(getattr(cmd, "tau", 0.0))
                kp    = float(getattr(cmd, "kp", 0.0))
                kd    = float(getattr(cmd, "kd", 0.0))
                qdes  = float(getattr(cmd, "q",  self.mj_data.sensordata[i]))
                dqdes = float(getattr(cmd, "dq", self.mj_data.sensordata[i + self.num_motor]))

                q  = float(self.mj_data.sensordata[i])
                dq = float(self.mj_data.sensordata[i + self.num_motor])

                u = tauff + kp * (qdes - q) + kd * (dqdes - dq)

                # clamp to actuator ctrl range if defined
                lo, hi = self.mj_model.actuator_ctrlrange[i]
                if lo < hi:
                    if u < lo: u = lo
                    elif u > hi: u = hi

                self.mj_data.ctrl[i] = u

            # For sim actuators beyond IDL length, zero them unless they are hands
            for i in range(N, N_sim):
                if i in self.hand_aids:
                    continue
                self.mj_data.ctrl[i] = 0.0

        except Exception:
            import traceback
            print("[LowCmdHandler] Exception:")
            traceback.print_exc()


    def _extract_positions(self, msg: HandCmd_):
        try:
            return [msg.motor_cmd[i].q for i in range(7)]
        except Exception:
            return None
    
    def _fill_hand_state_from_map(self, amap, hs: HandState_):
        for i in range(min(7, len(amap))):
            entry = amap[i]
            if not entry:
                hs.motor_state[i].q = 0.0
                hs.motor_state[i].dq = 0.0
                hs.motor_state[i].tau_est = 0.0
                continue
            aid  = entry["aid"]
            # motor sensors are ordered per actuator index: pos, vel, tau blocks
            q   = float(self.mj_data.sensordata[aid])
            dq  = float(self.mj_data.sensordata[aid + self.num_motor])
            tau = float(self.mj_data.sensordata[aid + 2 * self.num_motor])
            hs.motor_state[i].q = q
            hs.motor_state[i].dq = dq
            hs.motor_state[i].tau_est = tau


    def PublishHandState(self):
        if self.mj_data is None:
            return
        self._fill_hand_state_from_map(self.left_map, self.hand_state_L)
        for p in self.dex_L_state_publishers:
            p.Write(self.hand_state_L)

        self._fill_hand_state_from_map(self.right_map, self.hand_state_R)
        for p in self.dex_R_state_publishers:
            p.Write(self.hand_state_R)

    def DexLeftHandler(self, msg: HandCmd_):
        q = self._extract_positions(msg)
        # print("[DexLeft] q:", q)
        self._apply_hand_pd(self.left_map, q)
        print("[Dex map] L:", self.left_map)

    def DexRightHandler(self, msg: HandCmd_):
        q = self._extract_positions(msg)
        self._apply_hand_pd(self.right_map, q)


    def PublishLowState(self):
        try:
            if self.mj_data is None:
                return

            # motor_state is IDL-limited (35 on HG)
            N = min(self.num_motor, len(self.low_state.motor_state))
            for i in range(N):
                self.low_state.motor_state[i].q       = self.mj_data.sensordata[i]
                self.low_state.motor_state[i].dq      = self.mj_data.sensordata[i + self.num_motor]
                self.low_state.motor_state[i].tau_est = self.mj_data.sensordata[i + 2 * self.num_motor]

            # IMU/frame blocks: only touch if present (you already check have_* flags)
            if self.have_frame_sensor_:
                base = self.dim_motor_sensor
                # guard against short sensor vectors
                if base + 15 <= len(self.mj_data.sensordata):
                    self.low_state.imu_state.quaternion[0] = self.mj_data.sensordata[base + 0]
                    self.low_state.imu_state.quaternion[1] = self.mj_data.sensordata[base + 1]
                    self.low_state.imu_state.quaternion[2] = self.mj_data.sensordata[base + 2]
                    self.low_state.imu_state.quaternion[3] = self.mj_data.sensordata[base + 3]
                    self.low_state.imu_state.gyroscope[0]  = self.mj_data.sensordata[base + 4]
                    self.low_state.imu_state.gyroscope[1]  = self.mj_data.sensordata[base + 5]
                    self.low_state.imu_state.gyroscope[2]  = self.mj_data.sensordata[base + 6]
                    self.low_state.imu_state.accelerometer[0] = self.mj_data.sensordata[base + 7]
                    self.low_state.imu_state.accelerometer[1] = self.mj_data.sensordata[base + 8]
                    self.low_state.imu_state.accelerometer[2] = self.mj_data.sensordata[base + 9]

            if self.joystick != None:
                pygame.event.get()
                # Buttons
                self.low_state.wireless_remote[2] = int(
                    "".join(
                        [
                            f"{key}"
                            for key in [
                                0,
                                0,
                                int(self.joystick.get_axis(self.axis_id["LT"]) > 0),
                                int(self.joystick.get_axis(self.axis_id["RT"]) > 0),
                                int(self.joystick.get_button(self.button_id["SELECT"])),
                                int(self.joystick.get_button(self.button_id["START"])),
                                int(self.joystick.get_button(self.button_id["LB"])),
                                int(self.joystick.get_button(self.button_id["RB"])),
                            ]
                        ]
                    ),
                    2,
                )
                self.low_state.wireless_remote[3] = int(
                    "".join(
                        [
                            f"{key}"
                            for key in [
                                int(self.joystick.get_hat(0)[0] < 0),  # left
                                int(self.joystick.get_hat(0)[1] < 0),  # down
                                int(self.joystick.get_hat(0)[0] > 0), # right
                                int(self.joystick.get_hat(0)[1] > 0),    # up
                                int(self.joystick.get_button(self.button_id["Y"])),     # Y
                                int(self.joystick.get_button(self.button_id["X"])),     # X
                                int(self.joystick.get_button(self.button_id["B"])),     # B
                                int(self.joystick.get_button(self.button_id["A"])),     # A
                            ]
                        ]
                    ),
                    2,
                )
                # Axes
                sticks = [
                    self.joystick.get_axis(self.axis_id["LX"]),
                    self.joystick.get_axis(self.axis_id["RX"]),
                    -self.joystick.get_axis(self.axis_id["RY"]),
                    -self.joystick.get_axis(self.axis_id["LY"]),
                ]
                packs = list(map(lambda x: struct.pack("f", x), sticks))
                self.low_state.wireless_remote[4:8] = packs[0]
                self.low_state.wireless_remote[8:12] = packs[1]
                self.low_state.wireless_remote[12:16] = packs[2]
                self.low_state.wireless_remote[20:24] = packs[3]


            self.low_state_puber.Write(self.low_state)
        except Exception as e:
            import traceback; print("[PublishLowState] Exception:"); traceback.print_exc()
            
    def PublishHighState(self):

        if self.mj_data != None:
            self.high_state.position[0] = self.mj_data.sensordata[
                self.dim_motor_sensor + 10
            ]
            self.high_state.position[1] = self.mj_data.sensordata[
                self.dim_motor_sensor + 11
            ]
            self.high_state.position[2] = self.mj_data.sensordata[
                self.dim_motor_sensor + 12
            ]

            self.high_state.velocity[0] = self.mj_data.sensordata[
                self.dim_motor_sensor + 13
            ]
            self.high_state.velocity[1] = self.mj_data.sensordata[
                self.dim_motor_sensor + 14
            ]
            self.high_state.velocity[2] = self.mj_data.sensordata[
                self.dim_motor_sensor + 15
            ]

        self.high_state_puber.Write(self.high_state)

    def PublishWirelessController(self):
        if self.joystick != None:
            pygame.event.get()
            key_state = [0] * 16
            key_state[self.key_map["R1"]] = self.joystick.get_button(
                self.button_id["RB"]
            )
            key_state[self.key_map["L1"]] = self.joystick.get_button(
                self.button_id["LB"]
            )
            key_state[self.key_map["start"]] = self.joystick.get_button(
                self.button_id["START"]
            )
            key_state[self.key_map["select"]] = self.joystick.get_button(
                self.button_id["SELECT"]
            )
            key_state[self.key_map["R2"]] = (
                self.joystick.get_axis(self.axis_id["RT"]) > 0
            )
            key_state[self.key_map["L2"]] = (
                self.joystick.get_axis(self.axis_id["LT"]) > 0
            )
            key_state[self.key_map["F1"]] = 0
            key_state[self.key_map["F2"]] = 0
            key_state[self.key_map["A"]] = self.joystick.get_button(self.button_id["A"])
            key_state[self.key_map["B"]] = self.joystick.get_button(self.button_id["B"])
            key_state[self.key_map["X"]] = self.joystick.get_button(self.button_id["X"])
            key_state[self.key_map["Y"]] = self.joystick.get_button(self.button_id["Y"])
            key_state[self.key_map["up"]] = self.joystick.get_hat(0)[1] > 0
            key_state[self.key_map["right"]] = self.joystick.get_hat(0)[0] > 0
            key_state[self.key_map["down"]] = self.joystick.get_hat(0)[1] < 0
            key_state[self.key_map["left"]] = self.joystick.get_hat(0)[0] < 0

            key_value = 0
            for i in range(16):
                key_value += key_state[i] << i

            self.wireless_controller.keys = key_value
            self.wireless_controller.lx = self.joystick.get_axis(self.axis_id["LX"])
            self.wireless_controller.ly = -self.joystick.get_axis(self.axis_id["LY"])
            self.wireless_controller.rx = self.joystick.get_axis(self.axis_id["RX"])
            self.wireless_controller.ry = -self.joystick.get_axis(self.axis_id["RY"])

            self.wireless_controller_puber.Write(self.wireless_controller)

    def SetupJoystick(self, device_id=0, js_type="xbox"):
        pygame.init()
        pygame.joystick.init()
        joystick_count = pygame.joystick.get_count()
        if joystick_count > 0:
            self.joystick = pygame.joystick.Joystick(device_id)
            self.joystick.init()
        else:
            print("No gamepad detected.")
            sys.exit()

        if js_type == "xbox":
            self.axis_id = {
                "LX": 0,  # Left stick axis x
                "LY": 1,  # Left stick axis y
                "RX": 3,  # Right stick axis x
                "RY": 4,  # Right stick axis y
                "LT": 2,  # Left trigger
                "RT": 5,  # Right trigger
                "DX": 6,  # Directional pad x
                "DY": 7,  # Directional pad y
            }

            self.button_id = {
                "X": 2,
                "Y": 3,
                "B": 1,
                "A": 0,
                "LB": 4,
                "RB": 5,
                "SELECT": 6,
                "START": 7,
            }

        elif js_type == "switch":
            self.axis_id = {
                "LX": 0,  # Left stick axis x
                "LY": 1,  # Left stick axis y
                "RX": 2,  # Right stick axis x
                "RY": 3,  # Right stick axis y
                "LT": 5,  # Left trigger
                "RT": 4,  # Right trigger
                "DX": 6,  # Directional pad x
                "DY": 7,  # Directional pad y
            }

            self.button_id = {
                "X": 3,
                "Y": 4,
                "B": 1,
                "A": 0,
                "LB": 6,
                "RB": 7,
                "SELECT": 10,
                "START": 11,
            }
        else:
            print("Unsupported gamepad. ")

    def PrintSceneInformation(self):
        print(" ")

        print("<<------------- Link ------------->> ")
        for i in range(self.mj_model.nbody):
            name = mujoco.mj_id2name(self.mj_model, mujoco._enums.mjtObj.mjOBJ_BODY, i)
            if name:
                print("link_index:", i, ", name:", name)
        print(" ")

        print("<<------------- Joint ------------->> ")
        for i in range(self.mj_model.njnt):
            name = mujoco.mj_id2name(self.mj_model, mujoco._enums.mjtObj.mjOBJ_JOINT, i)
            if name:
                print("joint_index:", i, ", name:", name)
        print(" ")

        print("<<------------- Actuator ------------->>")
        for i in range(self.mj_model.nu):
            name = mujoco.mj_id2name(
                self.mj_model, mujoco._enums.mjtObj.mjOBJ_ACTUATOR, i
            )
            if name:
                print("actuator_index:", i, ", name:", name)
        print(" ")

        print("<<------------- Sensor ------------->>")
        index = 0
        for i in range(self.mj_model.nsensor):
            name = mujoco.mj_id2name(
                self.mj_model, mujoco._enums.mjtObj.mjOBJ_SENSOR, i
            )
            if name:
                print(
                    "sensor_index:",
                    index,
                    ", name:",
                    name,
                    ", dim:",
                    self.mj_model.sensor_dim[i],
                )
            index = index + self.mj_model.sensor_dim[i]
        print(" ")


class ElasticBand:

    def __init__(self):
        self.stiffness = 200
        self.damping = 100
        self.point = np.array([0, 0, 3])
        self.length = 0
        self.enable = True

    def Advance(self, x, dx):
        """
        Args:
          δx: desired position - current position
          dx: current velocity
        """
        δx = self.point - x
        distance = np.linalg.norm(δx)
        direction = δx / distance
        v = np.dot(dx, direction)
        f = (self.stiffness * (distance - self.length) - self.damping * v) * direction
        return f

    def MujuocoKeyCallback(self, key):
        glfw = mujoco.glfw.glfw
        if key == glfw.KEY_7:
            self.length -= 0.1
        if key == glfw.KEY_8:
            self.length += 0.1
        if key == glfw.KEY_9:
            self.enable = not self.enable
