from .roboclaw_3 import Roboclaw
from subprocess import run, PIPE

BUFFER_OR_INSTANT = 1  # 0 for buffer, 1 for instant write
GRIP_ACCEL = 20
POKER_ACCEL = 20

# Wrapper class for Roboclaw Motor Controller interfacing
class MCPMotorController:
    def __init__(self, dev_path_finder_file: str, bitrate: int):

        self.dev_path = self._get_motor_controller(dev_path_finder_file)

        if self.dev_path == None:
            raise Exception("MCP Motor controller not found.")
        
        self.rc = Roboclaw(self.dev_path, bitrate)
        self.address = 0x80
        self.rc.Open()

    def set_grip_velocity(self, move_dir: int):
        self.rc.SpeedAccelM1(self.address, GRIP_ACCEL, int(move_dir))

    def set_poker_velocity(self, move_dir: int):
        self.rc.SpeedAccelM2(self.address, POKER_ACCEL, int(move_dir))
    
    # Get motor controller device paths using serial IDs
    def _get_motor_controller(self, dev_path_finder_file: str) -> str | None:

        device_list = run(["\"" + dev_path_finder_file + "\""], 
                                     stdout=PIPE, text=True, 
                                     shell=True, executable='/bin/bash').stdout.splitlines()
        
        dev_path = None

        # Add device paths to devpath list
        for device in device_list:
            split_str = device.split(" - ")

            if "ID0001" in split_str[1]:
                dev_path = split_str[0]

        return dev_path

    