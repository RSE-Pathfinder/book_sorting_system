# ROS Python
import rclpy

# ROS Node Object
from rclpy.node import Node

# ROS Action Object
from rclpy.action import ActionServer, ActionClient

# Communication with User Interface
from bss_controller_interface.action import MoveArm

# Communication with UR10
from ros2_data.action import MoveXYZ

# Node
class ArmActionNode(Node):
    # Constructor
    def __init__(self):
        super().__init__('arm_action_node')
        
        # MoveArm Action Server Constructor
        self._action_server = ActionServer(
            self,
            MoveArm,                # ROS Action
            'bss_arm_action',       # ROS Topic
            self.execute_callback   # ROS Action Server Callback
            )
        
        # MoveXYZ Action Client Constructor
        self._action_client = ActionClient(
            self, 
            MoveXYZ,                # ROS Action
            'MoveXYZ'               # ROS Topic
            )
        
        self.get_logger().info('BSS Arm Action Node Ready')

    # MoveArm Action Server Callback
    def execute_callback(self, goal_handle):
        
        # Debug
        self.get_logger().info('Executing goal...')
        
        # Define MoveArm Action Feedback
        feedback = MoveArm.Feedback()
        feedback.feedback = 1
        
        index = goal_handle.request.index
        row = goal_handle.request.row
        col = goal_handle.request.col
        
        # Call MoveXYZ Action Client
        self.send_goal(0.8, 1.2, 1.2)
        
        # Indicate Goal Success
        goal_handle.succeed()
        
        # Define MoveArm Action Result
        result = MoveArm.Result()
        result.result = feedback.feedback
        return result
    
    # MoveXYZ Action Client Goal
    def send_goal(self, positionx, positiony, positionz):

        # Debug
        self.get_logger().info('Sending Goal...')

        goal_msg = MoveXYZ.Goal()
        goal_msg.positionx = positionx
        goal_msg.positiony = positiony
        goal_msg.positionz = positionz

        self._action_client.wait_for_server()
        
        # Returns a future to a goal handle
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        # Callback for when the future is complete
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    # MoveXYZ Action Client Response
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal Rejected')
            return
        
        self.get_logger().info('Goal Accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        
    #  MoveXYZ Action Client Feedback
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger.info('Result')
        
    arr = [
        # Origin
        [
            [
                [ 0.0, 0.0, 0.0 ], # Index 0, Row 0, Col 0
                [ 0.3, 0.3, 0.0 ]  # Index 0, Row 0, Col 1
            ]
        ],   
        # Shelf 1       
        [
            [
                [ 0.3, 0.2, 0.4 ], # Index 1, Row 0, Col 0
                [ 0.4, 0.3, 0.4 ]  # Index 1, Row 0, Col 1
            ],      
            [
                [ 0.3, 0.2, 0.8 ], # Index 1, Row 1, Col 0
                [ 0.3, 0.2, 0.8 ]  # Index 1, Row 1, Col 1
            ],      
        ]           
    ]


# Main
def main(args=None):
    rclpy.init(args=args)

    arm_action_node = ArmActionNode()

    rclpy.spin(arm_action_node)
    
    arm_action_node.destroy_node()
    
    rclpy.shutdown


if __name__ == '__main__':
    main()
    