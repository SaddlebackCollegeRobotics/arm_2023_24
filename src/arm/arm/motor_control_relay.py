import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from .motor_controller import MotorControllerManager, MotorController
from odrive.enums import AxisState

from threading import Thread
from signal import signal, SIGINT

from ament_index_python.packages import get_package_share_directory

class MinimalPublisher(Node):

    def __init__(self):

        # Give the node a name.
        super().__init__('motor_control_relay')

        # Signal handler for Ctrl+C
        # signal(SIGINT, self.signalHandler)

        self.control_input_subscriber = self.create_subscription(Float64MultiArray, '/arm/control_input', self.control_input_callback, 10)

        # Set up motor controllers ---------------------------------------

        self._max_speed = 3


        self._manager = MotorControllerManager(get_package_share_directory('arm') + '/flat_endpoints.json')
        
        # self._manager.add_motor_controller('azimuth', 4, self._max_speed)
        self._manager.add_motor_controller('shoulder', 5, self._max_speed)
        self._manager.add_motor_controller('elbow', 6, self._max_speed)
        # self._manager.add_motor_controller('pitch', 7, self._max_speed)
        # self._manager.add_motor_controller('yaw', 8, self._max_speed)
        # self._manager.add_motor_controller('roll', 9, self._max_speed)

        
        self._manager.for_each(MotorController.set_axis_state, AxisState.CLOSED_LOOP_CONTROL)

    def control_input_callback(self, msg: Float64MultiArray):

        # azimuth_vel, sholder_vel, elbow_vel, pitch_vel, yaw_vel, roll_vel, _, _ = msg.data
        shoulder_vel, elbow_vel = msg.data

        # self._manager['azimuth'].set_normalized_velocity(azimuth_vel)
        self._manager['shoulder'].set_normalized_velocity(shoulder_vel)
        self._manager['elbow'].set_normalized_velocity(elbow_vel)
        # self._manager['pitch'].set_normalized_velocity(pitch_vel)
        # self._manager['yaw'].set_normalized_velocity(yaw_vel)
        # self._manager['roll'].set_normalized_velocity(roll_vel)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
