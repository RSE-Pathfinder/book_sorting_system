#! /usr/bin/env python3

# Time library
import time

# ROS2 Python
import rclpy
from rclpy.node import Node                         # ROS2 Node Object
from rclpy.duration import Duration                 # ROS2 Duration Object
from rclpy.action import ActionServer, ActionClient # ROS2 Action Object
from rclpy.executors import SingleThreadedExecutor  # ROS2 Single-Threaded Executor Object
from rclpy.timer import Timer                       # ROS2 Timer Object

# Communication with User Interface
from bss_controller_interface.action import MoveArm 

# Communication with Robot Arm
from ros2_data.action import MoveXYZ    # Control Arm Position
from ros2_data.action import MoveYPR, MoveROT, MoveRP    # Control Arm Orientation

# Communication with Mobile Shelf
from geometry_msgs.msg import PoseStamped
from .submodules.robot_navigator import BasicNavigator, NavigationResult

# MoveXYZ Action Client Node
class MoveXYZActionClient(Node):
    # Constructor
    def __init__(self):
        
        # Construct Node
        super().__init__('movexyz_action_client')
        
        # Construct Action Client
        self._action_client = ActionClient(self, MoveXYZ, 'MoveXYZ')
        
        # Debug Statement
        self.get_logger().info('MoveXYZ Action Client Node Ready')

    # MoveXYZ Action Client Send Goal
    def send_goal(self, positionx, positiony, positionz):
        
        # Debug Statement
        self.get_logger().info('Sending MoveXYZ Goal...')
        
        # Initialise Action Goal
        goal_msg = MoveXYZ.Goal()
        goal_msg.positionx = positionx
        goal_msg.positiony = positiony
        goal_msg.positionz = positionz
        
        # Wait for Action Server
        self._action_client.wait_for_server()
        
        return self._action_client.send_goal_async(goal_msg)
        
        # Send Goal & Check Future
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
    # # MoveXYZ Action Client Feedback Callback
    # def feedback_callback(self, feedback_msg):
    #     feedback = feedback_msg.feedback
    #     self.get_logger.info('Received feedback: {0}'.format(feedback.feedback))
        
    # # MoveXYZ Action Client Goal Response Callback
    # def goal_response_callback(self, future):
        
    #     # Get Goal Handle for Result
    #     goal_handle = future.result()
        
    #     # Return if Send Goal Rejected
    #     if not goal_handle.accepted:
    #         self.get_logger().info('Goal Rejected')
    #         return
    #     self.get_logger().info('Goal Accepted')

    #     # Get Result
    #     self._get_result_future = goal_handle.get_result_async()
    #     self._get_result_future.add_done_callback(self.get_result_callback)
        
    # #  MoveXYZ Action Client Result Callback
    # def get_result_callback(self, future):
        
    #     # Initialise Result Variable
    #     result = future.result().result
        
    #     # Print Result(s)
    #     self.get_logger().info('MoveXYZ Result: {0}'.format(result.result))

# MoveYPR Action Client Node
class MoveYPRActionClient(Node):
    # Constructor
    def __init__(self):
        
        # Construct Node
        super().__init__('moveypr_action_client')
        
        # Construct Action Client
        self._action_client = ActionClient(self, MoveYPR, 'MoveYPR')
        
        # Debug Statement
        self.get_logger().info('MoveYPR Action Client Node Ready')

    # MoveYPR Action Client Send Goal
    def send_goal(self, yaw, pitch, roll):
        
        # Debug Statement
        self.get_logger().info('Sending MoveYPR Goal...')
        
        # Initialise Action Goal
        goal_msg = MoveYPR.Goal()
        goal_msg.yaw    = yaw
        goal_msg.pitch  = pitch
        goal_msg.roll   = roll
        
        # Wait for Action Server
        self._action_client.wait_for_server()
        
        return self._action_client.send_goal_async(goal_msg)
        
        # Send Goal & Check Future
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
    # # MoveXYZ Action Client Feedback Callback
    # def feedback_callback(self, feedback_msg):
    #     feedback = feedback_msg.feedback
    #     self.get_logger.info('Received feedback: {0}'.format(feedback.feedback))

    # # MoveYPR Action Client Response Callback
    # def goal_response_callback(self, future):
        
    #     # Get Goal Handler for Result
    #     goal_handle = future.result()
        
    #     # Return if Send Goal Rejected
    #     if not goal_handle.accepted:
    #         self.get_logger().info('MoveYPR Goal Rejected')
    #         return
    #     self.get_logger().info('MoveYPR Goal Accepted')

    #     # Get Result
    #     self._get_result_future = goal_handle.get_result_async()
    #     self._get_result_future.add_done_callback(self.get_result_callback)
        
    # #  MoveXYZ Action Client Result Callback
    # def get_result_callback(self, future):
        
    #     # Initialise Result Variable
    #     result = future.result().result
        
    #     # Print Result(s)
    #     self.get_logger().info('MoveYPR Result: {0}'.format(result.result))
    
# BSS Commander Node
class BSSCommanderNode(Node):
    # Constructor
    def __init__(self):
        
        # Create Node
        super().__init__('bss_commander_node')
        
        # User Interface MoveArm Action Server Constructor
        self._movearm_action_server = ActionServer(
            self,
            MoveArm,            # ROS Action
            'bss_arm_action',   # ROS Topic
            self.movearm_execute_callback   # ROS Action Server Callback
            )
        
        # Mobile Shelf Navigator Action Client Constructor
        self._navigator_action_client = BasicNavigator()
        
        # Robot Arm MoveXYZ Action Client Constructor
        self._movexyz_action_client = MoveXYZActionClient()
        
        # Robot Arm MoveYPR Action Client Constructor
        self._moveypr_action_client = MoveYPRActionClient()
        
        # Initialise Navigator Goal Pose
        _navigator_goal_pose = PoseStamped()
        _navigator_goal_pose.header.frame_id = 'map'
        _navigator_goal_pose.header.stamp = self._navigator_action_client.get_clock().now().to_msg()
        _navigator_goal_pose.pose.position.x = 0.0
        _navigator_goal_pose.pose.position.y = 0.0
        _navigator_goal_pose.pose.position.z = 0.0
        _navigator_goal_pose.pose.orientation.x = 0.0
        _navigator_goal_pose.pose.orientation.y = 0.0
        _navigator_goal_pose.pose.orientation.z = 0.0
        _navigator_goal_pose.pose.orientation.w = 0.0
        self._navigator_goal_pose_table.append(_navigator_goal_pose)
        # self.get_logger().info(self._navigator_goal_pose_table[0].header.frame_id)
        
        self.get_logger().info('BSS Arm Action Node Ready')

    # MoveArm Action Server Callback
    def movearm_execute_callback(self, goal_handle):
        
        # Debug
        self.get_logger().info('MoveArm Executing goal...')
        
        # Define MoveArm Action Feedback
        feedback = MoveArm.Feedback()
        feedback.feedback = [0, 1]
        
        # Initialise Action Variables
        index = goal_handle.request.index
        row = goal_handle.request.row
        col = goal_handle.request.col
        scoop_state = goal_handle.request.scoop_state
        
        # # Call Navigator Action Client
        # self._navigator_action_client.waitUntilNav2Active()
        
        # # Send PoseStamped Object
        # self._navigator_action_client.goToPose(goal_pose)
        # i = 0
        
        # # Blocking Loop
        # while not self._navigator_action_client.isNavComplete():
        #     i = i + 1
            
        #     # Feedback Handler
        #     feedback = self._navigator_action_client.getFeedback()
        #     if feedback and i % 5 == 0:
        #         self.get_logger().info('Distance Remaining: ' + '{:.2f}'.format(feedback.distance_remaining) + ' meters.')
            
        #         # Navigation Timeout >> Cancel Navigation
        #         if Duration.from_msg(feedback.navigation_time) > Duration(seconds=300.0):
        #             self._navigator_action_client.cancelNav()
                
        #         # Navigation Timeout >> Reset Position
        #         if Duration.from_msg(feedback.navigation_time) > Duration(seconds=180.0):
        #             goal_pose_alt = PoseStamped()
        #             goal_pose_alt.header.frame_id = 'map'
        #             goal_pose_alt.header.stamp = self._navigator_action_client.get_clock().now().to_msg()
        #             goal_pose_alt.pose.position.x = 0.0
        #             goal_pose_alt.pose.position.y = 0.0
        #             goal_pose_alt.pose.position.z = 0.0
        #             goal_pose_alt.pose.orientation.x = 0.0
        #             goal_pose_alt.pose.orientation.y = 0.0
        #             goal_pose_alt.pose.orientation.z = 0.0
        #             goal_pose_alt.pose.orientation.w = 0.0
        #             #self._navigator_action_client.goThroughPoses([goal_pose_alt])
            
        # # Get Result
        # result = self._navigator_action_client.getResult()
        # if result == NavigationResult.SUCCEEDED:
        #     self.get_logger().info('Mobile Shelf Goal Succeeded!')
        # elif result == NavigationResult.CANCELLED:
        #     self.get_logger().info('Mobile Shelf Goal Cancelled!')
        # elif result == NavigationResult.FAILED:
        #     self.get_logger().info('Mobile Shelf Goal Failed!')
        # else:
        #     self.get_logger().info('Mobile Shelf Unknown Error!')
        
        # Call MoveXYZ Action Client
        futurexyz = self._movexyz_action_client.send_goal(
            self._movexyz_goal_pose_table[index][row][col][0], # x-coordinate
            self._movexyz_goal_pose_table[index][row][col][1], # y-coordinate
            self._movexyz_goal_pose_table[index][row][col][2], # z-coordinate
            )
        rclpy.spin_until_future_complete(self._movexyz_action_client, futurexyz)
        
        # Call MoveYPR Action Client
        futureypr = self._moveypr_action_client.send_goal(
            self._moveypr_goal_pose_table[scoop_state][0],        # yaw
            self._moveypr_goal_pose_table[scoop_state][1],        # pitch
            self._moveypr_goal_pose_table[scoop_state][2],        # roll
            )
        rclpy.spin_until_future_complete(self._moveypr_action_client, futureypr)
        
        # Indicate Goal Success
        goal_handle.succeed()
        
        # Define MoveArm Action Result
        result = MoveArm.Result()
        result.result = feedback.feedback
        return result
    
    # Navigator Coordinate Table
    _navigator_goal_pose_table = []
        
    # MoveXYZ Coordinate Table
    _movexyz_goal_pose_table = [
        # Origin (Pickup and Dropoff Point)
        [
            [
                [ 0.0, 0.0, 0.0 ],  # Index 0, Row 0, Col 0
                [ 0.3, 0.3, 0.0 ]   # Index 0, Row 0, Col 1
            ]
        ],
        [   # Shelf Position 1
            [
                [ 0.3, 0.2, 0.8 ],  # Index 1, Row 0, Col 0
                [ 0.4, 0.3, 0.8 ]   # Index 1, Row 0, Col 1
            ],
            [
                [ 0.3, 0.2, 1.2 ],  # Index 1, Row 1, Col 0
                [ 0.3, 0.2, 1.2 ]   # Index 1, Row 1, Col 1
            ],
        ]
    ]
    
    # MoveYPR Coordinate Table
    _moveypr_goal_pose_table = [
        [ 180.0, 0.0, 090.0 ],    # Scoop Neutral
        [ 180.0, 0.0, 135.0 ],    # Scoop Droop
        [ 180.0, 0.0, 045.0 ]     # Scoop Catch
    ]

# Main
def main(args=None):
    rclpy.init(args=args)

    bss_commander_node = BSSCommanderNode()
    
    rclpy.spin(bss_commander_node)
    
    # Shut Down Nav2
    bss_commander_node._navigator_action_client.lifecycleShutdown()
    
    # Call Node Destructor
    bss_commander_node._movexyz_action_client.destroy_node()
    bss_commander_node._moveypr_action_client.destroy_node()
    bss_commander_node.destroy_node()
    
    # Shutdown ROS2
    rclpy.shutdown


if __name__ == '__main__':
    main()
    