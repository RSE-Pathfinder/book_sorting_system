import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from bss_controller_interface.action import MoveArm


class ArmActionServer(Node):
    
    def __init__(self):
        super().__init__('arm_action_server')
        self._action_server = ActionServer(
            self,
            MoveArm,
            'bss_arm_action',
            self.execute_callback)
        self.get_logger().info('BSS Arm Action Server Ready')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        goal_handle.succeed()
        
        result = MoveArm.Result()
        return result


def main(args=None):
    rclpy.init(args=args)

    arm_action_server = ArmActionServer()

    rclpy.spin(arm_action_server)
    
    arm_action_server.destroy_node()
    
    rclpy.shutdown


if __name__ == '__main__':
    main()
    