import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from ament_index_python.packages import get_package_share_directory

from .odrive_motor_controller_manager import ODriveMotorControllerManager
from .odrive_motor_controller import ODriveMotorController
from odrive.enums import AxisState

from .mcp_motor_controller import MCPMotorController

class MinimalPublisher(Node):

    def __init__(self):

        # Give the node a name.
        super().__init__('motor_control_relay')

        self.control_input_subscriber = self.create_subscription(Float64MultiArray, '/arm/control_input', self.control_input_callback, 10)

        # Set up motor controllers ---------------------------------------

        self._max_speed = 3

        self.mcp_controller = MCPMotorController(get_package_share_directory('arm') + '/find_devpath.bash', 115200)
        self._odrive_manager = ODriveMotorControllerManager(
            'can0', 
            get_package_share_directory('arm') + '/flat_endpoints.json',
            1000000)
        
        self._odrive_manager.add_motor_controller('azimuth', 4, self._max_speed)
        self._odrive_manager.add_motor_controller('bicep', 5, self._max_speed)
        self._odrive_manager.add_motor_controller('forearm', 6, self._max_speed)
        self._odrive_manager.add_motor_controller('pitch',  7, self._max_speed)
        self._odrive_manager.add_motor_controller('yaw', 8, self._max_speed)
        self._odrive_manager.add_motor_controller('roll', 9, self._max_speed)

        self._odrive_manager.for_each(ODriveMotorController.set_axis_state, AxisState.CLOSED_LOOP_CONTROL)


    def control_input_callback(self, msg: Float64MultiArray):

        azimuth_vel, bicep_vel, forearm_vel, pitch_vel, yaw_vel, roll_vel, grip_dir = msg.data

        self._odrive_manager['azimuth'].set_normalized_velocity(azimuth_vel)
        self._odrive_manager['bicep'].set_normalized_velocity(bicep_vel)
        self._odrive_manager['forearm'].set_normalized_velocity(forearm_vel)
        self._odrive_manager['pitch'].set_normalized_velocity(pitch_vel)
        self._odrive_manager['yaw'].set_normalized_velocity(yaw_vel)
        self._odrive_manager['roll'].set_normalized_velocity(roll_vel)
        
        self.mcp_controller.set_grip_velocity(int(grip_dir))



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
