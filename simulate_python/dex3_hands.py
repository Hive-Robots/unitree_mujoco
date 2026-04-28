import mujoco

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandState_ as HandState_default
from unitree_sdk2py.utils.thread import RecurrentThread


LEFT_NAMES = [
    "left_hand_thumb_0", "left_hand_thumb_1", "left_hand_thumb_2",
    "left_hand_middle_0", "left_hand_middle_1",
    "left_hand_index_0",  "left_hand_index_1",
]
RIGHT_NAMES = [
    "right_hand_thumb_0", "right_hand_thumb_1", "right_hand_thumb_2",
    "right_hand_middle_0", "right_hand_middle_1",
    "right_hand_index_0",  "right_hand_index_1",
]

LEFT_CMD_TOPICS  = ["rt/dex3/left/cmd",  "rt/g1_dex3_left_cmd",  "rt/g1/dex3/left/cmd",  "rt/dex3/left"]
RIGHT_CMD_TOPICS = ["rt/dex3/right/cmd", "rt/g1_dex3_right_cmd", "rt/g1/dex3/right/cmd", "rt/dex3/right"]
LEFT_STATE_TOPICS  = ["rt/dex3/left/state",  "rt/lf/dex3/left/state"]
RIGHT_STATE_TOPICS = ["rt/dex3/right/state", "rt/lf/dex3/right/state"]

KP = [12.0] * 7
KD = [0.6]  * 7


class Dex3Hands:
    """Bridge Unitree Dex3 hand DDS topics to MuJoCo actuators."""

    def __init__(self, mj_model, mj_data):
        self.mj_model = mj_model
        self.mj_data = mj_data
        self._nu = mj_model.nu

        self.left_map  = self._build_maps(LEFT_NAMES)
        self.right_map = self._build_maps(RIGHT_NAMES)

        self._state_L = HandState_default()
        self._state_R = HandState_default()

        self._state_pubs_L = [ChannelPublisher(t, HandState_) for t in LEFT_STATE_TOPICS]
        self._state_pubs_R = [ChannelPublisher(t, HandState_) for t in RIGHT_STATE_TOPICS]
        for p in self._state_pubs_L + self._state_pubs_R:
            p.Init()

        self._cmd_subs_L = []
        self._cmd_subs_R = []
        for t in LEFT_CMD_TOPICS:
            s = ChannelSubscriber(t, HandCmd_)
            s.Init(self._left_handler, 10)
            self._cmd_subs_L.append(s)
        for t in RIGHT_CMD_TOPICS:
            s = ChannelSubscriber(t, HandCmd_)
            s.Init(self._right_handler, 10)
            self._cmd_subs_R.append(s)

        self._thread = RecurrentThread(
            interval=mj_model.opt.timestep,
            target=self._publish_states,
            name="sim_dex3_state",
        )
        self._thread.Start()

        print(f"Dex3Hands: left={[m is not None for m in self.left_map]}, "
              f"right={[m is not None for m in self.right_map]}")

    def _build_maps(self, names):
        maps = []
        for n in names:
            aid = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
            if aid < 0:
                maps.append(None)
                continue
            jnt_id = int(self.mj_model.actuator_trnid[aid, 0])
            if jnt_id < 0:
                maps.append(None)
                continue
            maps.append({
                "aid":  aid,
                "qadr": int(self.mj_model.jnt_qposadr[jnt_id]),
                "dadr": int(self.mj_model.jnt_dofadr[jnt_id]),
            })
        return maps

    def _apply_pd(self, amap, q_des):
        if not q_des:
            return
        for i, entry in enumerate(amap[:len(q_des)]):
            if not entry:
                continue
            aid = entry["aid"]
            q   = float(self.mj_data.qpos[entry["qadr"]])
            dq  = float(self.mj_data.qvel[entry["dadr"]])
            tau = KP[i] * (q_des[i] - q) - KD[i] * dq
            lo, hi = self.mj_model.actuator_ctrlrange[aid]
            if lo < hi:
                tau = max(lo, min(hi, tau))
            self.mj_data.ctrl[aid] = tau

    def _extract_q(self, msg):
        try:
            return [msg.motor_cmd[i].q for i in range(7)]
        except Exception:
            return None

    def _fill_state(self, amap, hs):
        for i, entry in enumerate(amap[:7]):
            if not entry:
                hs.motor_state[i].q = 0.0
                hs.motor_state[i].dq = 0.0
                hs.motor_state[i].tau_est = 0.0
                continue
            aid = entry["aid"]
            hs.motor_state[i].q       = float(self.mj_data.sensordata[aid])
            hs.motor_state[i].dq      = float(self.mj_data.sensordata[aid + self._nu])
            hs.motor_state[i].tau_est = float(self.mj_data.sensordata[aid + 2 * self._nu])

    def _left_handler(self, msg: HandCmd_):
        try:
            self._apply_pd(self.left_map, self._extract_q(msg))
        except Exception as e:
            print("[Dex3Hands left] error:", e)

    def _right_handler(self, msg: HandCmd_):
        try:
            self._apply_pd(self.right_map, self._extract_q(msg))
        except Exception as e:
            print("[Dex3Hands right] error:", e)

    def _publish_states(self):
        if self.mj_data is None:
            return
        self._fill_state(self.left_map, self._state_L)
        for p in self._state_pubs_L:
            p.Write(self._state_L)
        self._fill_state(self.right_map, self._state_R)
        for p in self._state_pubs_R:
            p.Write(self._state_R)
