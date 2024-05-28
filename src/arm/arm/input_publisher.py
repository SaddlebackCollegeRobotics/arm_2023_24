import os
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty
from ament_index_python.packages import get_package_share_directory

from . import gamepad_input as gmi
import numpy as np


class InputPublisher(Node):

    def __init__(self):

        # Give the node a name.
        super().__init__('arm_input_publisher')

        self.PUBLISHER_PERIOD = 1/10 # seconds
        self.PRECISION_FACTOR = 0.25

        self.control_publisher = self.create_publisher(Float64MultiArray, '/arm/control_input', 10)
        self.reset_arm_cli = self.create_client(Empty, '/arm/reset_system')

        self.msg = Float64MultiArray()
        self.reset_arm_request = Empty.Request()

        self.timer = self.create_timer(self.PUBLISHER_PERIOD, self.timer_callback)

        self.AXIS_DEADZONE = 0.1

        gmi.setConfigFile(get_package_share_directory('arm') + '/gamepads.config')
        gmi.run_event_loop()

        self.reset_arm_pressed = False

    def timer_callback(self):
        
        gamepad = gmi.getGamepad(1)

        # Trigger press acts as a safety switch as well as a workaround
        # for a pygame bug that causes specifically 8bitdo controllers to
        # register a constant input of 1.0 and -1.0 on joystick axes, when
        # unplugged and plugged back in.
        if gamepad != None and gmi.getTriggers(gamepad, self.AXIS_DEADZONE)[1] > 0:

            (ls_x, ls_y) = gmi.getLeftStick(gamepad, self.AXIS_DEADZONE)
            (rs_x, rs_y) = gmi.getRightStick(gamepad, self.AXIS_DEADZONE)
            (x_hat, y_hat) = gmi.getHat(gamepad)

            if gmi.getButtonValue(gamepad, 2): # south
                grip_dir = -1.0
            elif gmi.getButtonValue(gamepad, 3): # east
                grip_dir = 1.0
            else:
                grip_dir = 0.0

            if gmi.getButtonValue(gamepad, 8): #r1
                ls_y = 0.0
                rs_y = 0.0
            else:
                ls_x = 0.0
                rs_x = 0.0

            # Press Share and Options (+ and -) to reset arm system
            if gmi.getButtonValue(gamepad, 4) and gmi.getButtonValue(gamepad, 5):
                
                if self.reset_arm_pressed == False:
                    self.reset_arm_pressed = True
                    self.reset_arm()
            else:
                self.reset_arm_pressed = False

            controls_array = [float(e) for e in [ls_x, -ls_y, rs_y,
            y_hat, -rs_x, x_hat]]

            if gmi.getButtonValue(gamepad, 7):
                controls_array = list(map(lambda e: self.PRECISION_FACTOR * e, controls_array))
                print("Precise Input Mode: ", controls_array) 
    
            controls_array.append(grip_dir)

            # Azimuth, bicep, forearm, pitch, yaw, roll, grip_dir, enable_precision_mode
            try:
                self.msg.data = controls_array
            except:
                print("Invalid controls_array!", controls_array)
        else:
            self.msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.control_publisher.publish(self.msg)

    def reset_arm(self):
    
        if (self.reset_arm_cli.service_is_ready() == False):
            print("Warning: Arm reset service is unavailable!")
            return
        
        self.future = self.reset_arm_cli.call_async(self.reset_arm_request)
        self.future.add_done_callback(self.reset_arm_callback)        

    def reset_arm_callback(self, future):
        if (future.result() != None):
            print("Successfully reset arm system!")
        else:
            print("Warning: Failed to reset arm system!")


def main(args=None):
    rclpy.init(args=args)

    input_publisher = InputPublisher()

    rclpy.spin(input_publisher)

    # Destroy the node explicitly
    input_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
