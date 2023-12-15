import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory

from .motor_controller import MotorControllerManager

import ifcfg

from . import gamepad_input
import os
import subprocess


class MinimalPublisher(Node):

    def __init__(self):

        # Give the node a name.
        super().__init__('minimal_publisher')

        self.package_share_directory = get_package_share_directory('py_pubsub')

        # Check for CAN bus connection
        self.configure_can_bus(interface='can0', bitrate=1000000)
        
        # Set up motor controllers ---------------------------------------

        INITIAL_MAX_SPEED = 30

        self.motor_controller_manager = MotorControllerManager(self)

        for i in range(4, 9):
            self.motor_controller_manager.add_motor_controller(node_id=i, max_speed=INITIAL_MAX_SPEED)

        # --------------------------------------------------------------------------

        # Set up gamepad input
        self.configure_gamepad_input()

        timer_period = 1/10 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        
    def timer_callback(self):
        
        gamepad = gamepad_input.getGamepad(0)

        if gamepad != None:
            (ls_x, ls_y) = gamepad_input.getLeftStick(gamepad, self.gamepad_deadzone)
            (rs_x, rs_y) = gamepad_input.getRightStick(gamepad, self.gamepad_deadzone)
            (l2, r2) = gamepad_input.getTriggers(gamepad, self.gamepad_deadzone)

            # TODO - Implement arm controls

        else:
            self.motor_controller_manager.set_velocity_all(0)

    def hatNorth(self):
        self.motor_controller_manager.change_max_speed_all(5)
    
    def hatSouth(self):
        self.motor_controller_manager.change_max_speed_all(-5)

    def configure_gamepad_input(self):
        
        self.gamepad_deadzone = 0.1

        config_path = os.path.join(self.package_share_directory, 'gamepads.config')
        gamepad_input.setConfigFile(config_path)

        hatEvents = [self.hatNorth, self.hatSouth, None, None, None]
        gamepad_input.run_event_loop(hatEvents=hatEvents)

    def configure_can_bus(self, interface: str, bitrate: int):

        found = False
        for name, _ in ifcfg.interfaces().items():
            if name == interface:
                found = True
                self.get_logger().info("CAN bus found")
                break  

        if not found:  
            self.get_logger().info("CAN bus not found")
            exit(0)

        subprocess.run(["sudo", "ip", "link", "set", interface, "up", "type", "can", "bitrate", str(bitrate)])


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
