import time
import mujoco
import mujoco.viewer
from threading import Thread
import threading

import numpy as np

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py_bridge import UnitreeSdk2Bridge, ElasticBand

import config


locker = threading.Lock()

mj_model = mujoco.MjModel.from_xml_path(config.ROBOT_SCENE)
mj_data  = mujoco.MjData(mj_model)


# ── Elastic band ──────────────────────────────────────────────────────────────
# The default anchor at (0, 0, 3) holds the torso ~1.7 m above its spawn,
# making the robot float in the air. Lower the anchor so the suspended torso
# sits at a height where the feet are at floor level.
#
# Rest length is set to -0.3 (≡ 3 presses of key 7 at startup), which adds
# 60 N of pre-tension on top of the geometric stretch force and raises the
# equilibrium height so the feet clear the floor.

if config.ENABLE_ELASTIC_BAND:
    elastic_band = ElasticBand()

    # Lower the anchor: torso world-z at spawn ≈ 1.30 m. Anchor at 1.55 m
    # gives ~25 cm of stretch, which with the default stiffness=200 produces
    # ~50 N upward — enough to keep the body suspended just above the floor
    # without lifting it back into the air.
    elastic_band.point = np.array([0.0, 0.0, 1.55])

    # Pre-tension equivalent to 3× key-7 presses: raises equilibrium ~0.3 m.
    elastic_band.length = -0.3

    if config.ROBOT == "h1" or config.ROBOT == "g1":
        band_attached_link = mj_model.body("torso_link").id
    else:
        band_attached_link = mj_model.body("base_link").id

    viewer = mujoco.viewer.launch_passive(
        mj_model, mj_data, key_callback=elastic_band.MujuocoKeyCallback
    )
else:
    viewer = mujoco.viewer.launch_passive(mj_model, mj_data)

mj_model.opt.timestep = config.SIMULATE_DT
num_motor_         = mj_model.nu
dim_motor_sensor_  = 3 * num_motor_
time.sleep(0.2)


def SimulationThread():
    global mj_data, mj_model
    ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)
    unitree = UnitreeSdk2Bridge(mj_model, mj_data)

    if config.USE_JOYSTICK:
        unitree.SetupJoystick(device_id=0, js_type=config.JOYSTICK_TYPE)
    if config.PRINT_SCENE_INFORMATION:
        unitree.PrintSceneInformation()

    while viewer.is_running():
        step_start = time.perf_counter()
        locker.acquire()

        if config.ENABLE_ELASTIC_BAND:
            if elastic_band.enable:
                mj_data.xfrc_applied[band_attached_link, :3] = elastic_band.Advance(
                    mj_data.qpos[:3], mj_data.qvel[:3]
                )

        mujoco.mj_step(mj_model, mj_data)
        locker.release()

        time_until_next_step = mj_model.opt.timestep - (
            time.perf_counter() - step_start
        )
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)


def PhysicsViewerThread():
    while viewer.is_running():
        locker.acquire()
        viewer.sync()
        locker.release()
        time.sleep(config.VIEWER_DT)


if __name__ == "__main__":
    viewer_thread = Thread(target=PhysicsViewerThread)
    sim_thread    = Thread(target=SimulationThread)

    viewer_thread.start()
    sim_thread.start()