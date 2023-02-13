import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from bss_controller_interface.action import MoveArm


class ArmActionClient(Node):
    
    def __init__(self):
        super().__init__('arm_action_client')
        self._action_client = ActionClient(
            self,
            MoveArm,
            'bss_arm_action')
        self.get_logger().info('BSS Arm Action Client Ready')

    def send_goal(self, index, row, col, drop):
        self.get_logger().info('Sending goal...')
        
        goal_msg = MoveArm.Goal()
        goal_msg.index = index
        goal_msg.row = row
        goal_msg.col = col
        goal_msg.drop = drop
        
        self._action_client.wait_for_server()
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        
        self.get_logger().info('Result: {0}'.format(result.result))


def main(args=None):
    rclpy.init(args=args)

    arm_action_client = ArmActionClient()

    arm_action_client.send_goal(1, 0, 1, 1)
    
    rclpy.spin(arm_action_client)
    
    arm_action_client.destroy_node()
    
    rclpy.shutdown


if __name__ == '__main__':
    main()
    