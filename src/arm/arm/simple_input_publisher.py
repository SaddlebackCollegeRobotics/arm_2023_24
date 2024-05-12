import os
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from ament_index_python.packages import get_package_share_directory

from . import gamepad_input as gmi
import numpy as mp

class InputPublisher(Node):

    def __init__(self):

        # Give the node a name.
        super().__init__('arm_input_publisher')

        self.PUBLISHER_PERIOD = 1/10 # seconds
        self.PRECISION_FACTOR = 0.5

        self.control_publisher = self.create_publisher(Float64MultiArray, '/arm/control_input', 10)
        self.msg = Float64MultiArray()

        self.timer = self.create_timer(self.PUBLISHER_PERIOD, self.timer_callback)

        self.AXIS_DEADZONE = 0.1
        gmi.setConfigFile(get_package_share_directory('arm') + '/gamepads.config')
        gmi.run_event_loop()

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
            
            controls_array = mp.array([ls_x, -ls_y, rs_y,
            y_hat, -rs_x, x_hat]).astype(float, copy = False) 

            if gmi.getButtonValue(gamepad, 7):
                controls_array *= self.PRECISION_FACTOR

            controls_array.append(grip_dir)

            # Azimuth, bicep, forearm, pitch, yaw, roll, grip_dir, enable_precision_mode
            self.msg.data = controls_array.tolist()
        else:
            self.msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.control_publisher.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)

    input_publisher = InputPublisher()

    rclpy.spin(input_publisher)

    # Destroy the node explicitly
    input_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
