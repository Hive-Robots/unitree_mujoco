import time
import sys
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from dataclasses import dataclass

@dataclass
class TestOption:
    name: str
    id: int

option_list = [
    TestOption("damp", 0),
    TestOption("Squat2StandUp", 1),
    TestOption("StandUp2Squat", 2),
    TestOption("move forward", 3),
    TestOption("move lateral", 4),
    TestOption("move rotate", 5),
    TestOption("low stand", 6),
    TestOption("high stand", 7),
    TestOption("zero torque", 8),
    TestOption("wave hand1", 9),
    TestOption("wave hand2", 10),
    TestOption("shake hand", 11),
    TestOption("Lie2StandUp", 12),
]

class UserInterface:
    def __init__(self):
        self.test_option = TestOption(name=None, id=None)

    def terminal_handle(self):
        input_str = input("Enter ID or name (or 'list'): ").strip()

        if input_str == "list":
            for opt in option_list:
                print(f"{opt.name:15s} | ID: {opt.id}")
            return None

        for opt in option_list:
            if input_str == opt.name or input_str == str(opt.id):
                self.test_option = opt
                print(f"> Selected: {opt.name} (ID: {opt.id})")
                return opt

        print("No matching test option found.")
        return None

if __name__ == "__main__":
    # Initialize DDS with loopback
    if len(sys.argv) < 2:
        ChannelFactoryInitialize(1, "lo")  # Use loopback interface
    else:
        ChannelFactoryInitialize(0, sys.argv[1])  # Custom interface

    print("⚠️ Ensure MuJoCo bridge is running. Press Enter to continue...")
    input()

    sport_client = LocoClient()
    sport_client.SetTimeout(10.0)
    sport_client.Init()

    ui = UserInterface()

    print("\n✅ Type 'list' to view available actions.")
    while True:
        choice = ui.terminal_handle()
        if choice is None:
            continue

        id = ui.test_option.id
        if id == 0:
            sport_client.Damp()
        elif id == 1:
            sport_client.Damp()
            time.sleep(0.5)
            sport_client.Squat2StandUp()
        elif id == 2:
            sport_client.StandUp2Squat()
        elif id == 3:
            sport_client.Move(0.3, 0, 0)
        elif id == 4:
            sport_client.Move(0, 0.3, 0)
        elif id == 5:
            sport_client.Move(0, 0, 0.3)
        elif id == 6:
            sport_client.LowStand()
        elif id == 7:
            sport_client.HighStand()
        elif id == 8:
            sport_client.ZeroTorque()
        elif id == 9:
            sport_client.WaveHand()
        elif id == 10:
            sport_client.WaveHand(True)
        elif id == 11:
            sport_client.ShakeHand()
            time.sleep(3)
            sport_client.ShakeHand()
        elif id == 12:
            sport_client.Damp()
            time.sleep(0.5)
            sport_client.Lie2StandUp()

        time.sleep(1)
