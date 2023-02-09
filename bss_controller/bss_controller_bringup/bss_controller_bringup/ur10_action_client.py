import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from ros2_data.action import MoveXYZ

class UR10ActionClient(Node): 

    def __init__(self):
        super().__init__('ur10_action_client')
        self._action_client = ActionClient(self, MoveXYZ, 'MoveXYZ')

        #Debugging Statement
        self.get_logger().info('UR10 Action Client Ready')

    #Ros2 node to add the action client to self, row and col are from MoveArm.action
    def send_goal(self, positionx, positiony, positionz):

        #Debugging Statement
        self.get_logger().info('Sending Goal...')

        goal_msg = MoveXYZ.Goal()
        goal_msg.positionx = positionx
        goal_msg.positiony = positiony
        goal_msg.positionz = positionz

        self._action_client.wait_for_server()
        
        #Returns a future to a goal handle
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        #Callback for when the future is complete
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal Rejected')
            return
        
        self.get_logger().info('Goal Accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger.info('Result')
        
       


def main(args=None):
    rclpy.init(args=args)

    
    ur10_action_client = UR10ActionClient() 

    ur10_action_client.send_goal(1.0, 2.0, 3.0)

    rclpy.spin(ur10_action_client)

    ur10_action_client.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
