import os
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from ament_index_python.packages import get_package_share_directory

from . import gamepad_input as gmi

class InputPublisher(Node):

    def __init__(self):

        # Give the node a name.
        super().__init__('arm_input_publisher')

        self.PUBLISHER_PERIOD = 1/10 # seconds

        self.control_publisher = self.create_publisher(Float64MultiArray, '/arm/control_input', 10)
        self.msg = Float64MultiArray()

        self.timer = self.create_timer(self.PUBLISHER_PERIOD, self.timer_callback)

        self.AXIS_DEADZONE = 0.1
        gmi.setConfigFile(get_package_share_directory('arm') + '/gamepads.config')
        gmi.run_event_loop()

    def timer_callback(self):
        
        gamepad = gmi.getGamepad(0)

        if gamepad != None:

            (ls_x, ls_y) = gmi.getLeftStick(gamepad, self.AXIS_DEADZONE)
            (rs_x, rs_y) = gmi.getRightStick(gamepad, self.AXIS_DEADZONE)
            self.msg.data = [float(ls_y), float(rs_y)]
        else:
            self.msg.data = [0.0, 0.0]

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
