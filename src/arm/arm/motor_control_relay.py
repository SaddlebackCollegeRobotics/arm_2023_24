import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String, Empty as EmptyMsg
from std_srvs.srv import Empty as EmptySrv
from ament_index_python.packages import get_package_share_directory

from .odrive_motor_controller_manager import ODriveMotorControllerManager
from .odrive_motor_controller import ODriveMotorController
from odrive.enums import AxisState

from .mcp_motor_controller import MCPMotorController

from time import time

class MotorControlRelay(Node):

    def __init__(self):

        # Give the node a name.
        super().__init__('arm_motor_control_relay')

        self.control_input_subscriber = self.create_subscription(Float64MultiArray, '/arm/control_input', self.control_input_callback, 10)
        self._heartbeat_subscriber = self.create_subscription(EmptyMsg, '/system/heartbeat', self.reset_heartbeat, 10)
        self._status_publisher = self.create_publisher(String, '/arm/status', 10)
        self.reset_arm_srv = self.create_service(EmptySrv, '/arm/reset_system', self.reset_odrives)

        # Listen and react to system heartbeat ----------------------
        self.HEARTBEAT_TIMEOUT = 2.0
        self.HEARTBEAT_CHECK_PERIOD = 0.5

        self.last_heartbeat_time = 0
        self.is_heartbeat_active = False
        self.heartbeat_check_ = self.create_timer(self.HEARTBEAT_CHECK_PERIOD, self.check_heartbeat)

        # Feed the motor controller watchdogs -----------------------
        self.WATCHDOG_FEED_PERIOD = 0.5
        self.watchdog_feed_timer = self.create_timer(self.WATCHDOG_FEED_PERIOD, self.feed_watchdogs)
        
        # Arm system status ---------------------------------------
        self.ARM_STATUS_PERIOD = 5
        self.arm_status_timer = self.create_timer(self.ARM_STATUS_PERIOD, self.publish_arm_status)
        self.arm_status_msg = String()

        # Set up motor controllers ---------------------------------------

        self.MAX_SPEED = 3

        self.mcp_controller = MCPMotorController(get_package_share_directory('arm') + '/find_devpath.bash', 115200)
        self._odrive_manager = ODriveMotorControllerManager(
            'can0', 
            get_package_share_directory('arm') + '/flat_endpoints.json',
            1000000)
        
        self._odrive_manager.add_motor_controller('azimuth', 4, self.MAX_SPEED)
        self._odrive_manager.add_motor_controller('bicep', 5, self.MAX_SPEED * 3)
        self._odrive_manager.add_motor_controller('forearm', 6, self.MAX_SPEED)
        self._odrive_manager.add_motor_controller('pitch',  7, self.MAX_SPEED)
        self._odrive_manager.add_motor_controller('yaw', 8, self.MAX_SPEED)
        self._odrive_manager.add_motor_controller('roll', 9, self.MAX_SPEED)

        self._odrive_manager.for_each(ODriveMotorController.set_axis_state, AxisState.CLOSED_LOOP_CONTROL)

    def control_input_callback(self, msg: Float64MultiArray):

        if (self.is_heartbeat_active):
        
            azimuth_vel, bicep_vel, forearm_vel, pitch_vel, yaw_vel, roll_vel, grip_dir, poker_dir = msg.data

            self._odrive_manager['azimuth'].set_normalized_velocity(-azimuth_vel)
            self._odrive_manager['bicep'].set_normalized_velocity(-bicep_vel)
            self._odrive_manager['forearm'].set_normalized_velocity(forearm_vel)
            self._odrive_manager['pitch'].set_normalized_velocity(-pitch_vel)
            self._odrive_manager['yaw'].set_normalized_velocity(yaw_vel)
            self._odrive_manager['roll'].set_normalized_velocity(roll_vel)
            
            self.mcp_controller.set_grip_velocity(int(grip_dir))
            self.mcp_controller.set_poker_velocity(int(poker_dir))

    def reset_odrives(self, request, response):
        """Resets the odrives to closed loop control in case of
        a fatal error such as over-current.
        """
        self._odrive_manager.for_each(ODriveMotorController.clear_errors)
        self._odrive_manager.for_each(ODriveMotorController.set_axis_state, AxisState.CLOSED_LOOP_CONTROL)
        
        return response

    def reset_heartbeat(self, msg):
        self.last_heartbeat_time = time()

    def check_heartbeat(self):
        
        self.is_heartbeat_active = time() - self.last_heartbeat_time < self.HEARTBEAT_TIMEOUT
        
        print(self.is_heartbeat_active)

        if (self.is_heartbeat_active == False):
            self._odrive_manager.for_each(ODriveMotorController.set_normalized_velocity, 0.0)

    def feed_watchdogs(self):
        self._odrive_manager.for_each(ODriveMotorController.feed_watchdog)

    def publish_drive_status(self):
        self.arm_status_msg.data = self._odrive_manager.get_all_odrive_status()
        self._status_publisher.publish(self.arm_status_msg)


def main(args=None):
    rclpy.init(args=args)

    motor_control_relay = MotorControlRelay()

    rclpy.spin(motor_control_relay)

    # Destroy the node explicitly
    motor_control_relay.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
