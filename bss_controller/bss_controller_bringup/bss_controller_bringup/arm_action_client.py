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

    def send_goal(self, index, row, col, scoop_state):
        
        self.get_logger().info('Sending goal...')
        
        goal_msg = MoveArm.Goal()
        goal_msg.index = index
        goal_msg.row = row
        goal_msg.col = col
        goal_msg.scoop_state = scoop_state
        
        self._action_client.wait_for_server()
        
        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    arm_action_client = ArmActionClient()

    future = arm_action_client.send_goal(1, 0, 1, 1)
    
    rclpy.spin_until_future_complete(arm_action_client, future)
    
    arm_action_client.destroy_node()

if __name__ == '__main__':
    main()
    