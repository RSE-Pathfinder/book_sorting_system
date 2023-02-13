#! /usr/bin/env python3

# Time library
import time

# ROS2 Python
import rclpy
from rclpy.node import Node                         # ROS2 Node Object
from rclpy.duration import Duration                 # ROS2 Duration Object
from rclpy.action import ActionServer, ActionClient # ROS2 Action Object

# Communication with User Interface
from bss_controller_interface.action import MoveArm

# Communication with Robot Arm
from ros2_data.action import MoveXYZ
from ros2_data.action import MoveYPR

# Communication with Mobile Shelf
from geometry_msgs.msg import PoseStamped
from robot_navigator import BasicNavigator, NavigationResult

# Node
class BSSCommanderNode(Node):
    # Constructor
    def __init__(self):
        super().__init__('bss_commander_node')
        
        # User Interface MoveArm Action Server Constructor
        self._movearm_action_server = ActionServer(
            self,
            MoveArm,                # ROS Action
            'bss_arm_action',       # ROS Topic
            self.movearm_execute_callback   # ROS Action Server Callback
            )
        
        # Mobile Shelf Navigator Action Client Constructor
        self._navigator_action_client = BasicNavigator()
        
        # Robot Arm MoveXYZ Action Client Constructor
        self._movexyz_action_client = ActionClient(
            self, 
            MoveXYZ,                # ROS Action
            'MoveXYZ'               # ROS Topic
            )
        
        # Robot Arm MoveYPR Action Client Constructor
        self._moveypr_action_client = ActionClient(
            self, 
            MoveYPR,                # ROS Action
            'MoveYPR'               # ROS Topic
            )
        
        self.get_logger().info('BSS Arm Action Node Ready')

    # MoveArm Action Server Callback
    def movearm_execute_callback(self, goal_handle):
        
        # Debug
        self.get_logger().info('Executing goal...')
        
        # Define MoveArm Action Feedback
        feedback = MoveArm.Feedback()
        feedback.feedback = [0, 1]
        
        # Initialise Action Variables
        index = goal_handle.request.index
        row = goal_handle.request.row
        col = goal_handle.request.col
        drop = goal_handle.request.drop
        
        # Call Navigator Action Client
        self._navigator_action_client.waitUntilNav2Active()
        
        # Create PoseStamped Object
        goal_poses = []
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self._navigator_action_client.get_clock().now().to_msg()
        goal_pose.pose.position.x = 0.0
        goal_pose.pose.position.y = 0.0
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 0.0
        goal_poses.append(goal_pose)
        
        # Send PoseStamped Object
        self._navigator_action_client.goToPose(goal_pose)
        i = 0
        
        # Blocking Loop
        while not self._navigator_action_client.isNavComplete():
            i = i + 1
            
            # Feedback Handler
            feedback = self._navigator_action_client.getFeedback()
            if feedback and i % 5 == 0:
                self.get_logger().info('Distance Remaining: ' + '{:.2f}'.format(feedback.distance_remaining) + ' meters.')
            
                # Navigation Timeout >> Cancel Navigation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=300.0):
                    self._navigator_action_client.cancelNav()
                
                # Navigation Timeout >> Reset Position
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=180.0):
                    goal_pose_alt = PoseStamped()
                    goal_pose_alt.header.frame_id = 'map'
                    goal_pose_alt.header.stamp = self._navigator_action_client.get_clock().now().to_msg()
                    goal_pose_alt.pose.position.x = 0.0
                    goal_pose_alt.pose.position.y = 0.0
                    goal_pose_alt.pose.position.z = 0.0
                    goal_pose_alt.pose.orientation.x = 0.0
                    goal_pose_alt.pose.orientation.y = 0.0
                    goal_pose_alt.pose.orientation.z = 0.0
                    goal_pose_alt.pose.orientation.w = 0.0
                    self._navigator_action_client.goThroughPoses([goal_pose_alt])
            
        # Get Result
        result = self._navigator_action_client.getResult()
        if result == NavigationResult.SUCCEEDED:
            self.get_logger().info('Mobile Shelf Goal Succeeded!')
        elif result == NavigationResult.CANCELLED:
            self.get_logger().info('Mobile Shelf Goal Cancelled!')
        elif result == NavigationResult.FAILED:
            self.get_logger().info('Mobile Shelf Goal Failed!')
        else:
            self.get_logger().info('Mobile Shelf Unknown Error!')
        
        # Call MoveXYZ Action Client
        self.movexyz_send_goal(
            self._armXYZ[index][row][col][0], # x-coordinate
            self._armXYZ[index][row][col][1], # y-coordinate
            self._armXYZ[index][row][col][2], # z-coordinate
            )
        
        # Call MoveYPR Action Client
        self.moveypr_send_goal(
            self._armYPR[drop]                # neutral-drop-catch
            )
        
        # Indicate Goal Success
        goal_handle.succeed()
        
        # Define MoveArm Action Result
        result = MoveArm.Result()
        result.result = feedback.feedback
        return result
    
    # MoveXYZ Action Client Goal
    def movexyz_send_goal(self, positionx, positiony, positionz):

        # Debug
        self.get_logger().info('Sending Goal...')

        # Initial Action Variables
        goal_msg = MoveXYZ.Goal()
        goal_msg.positionx = positionx
        goal_msg.positiony = positiony
        goal_msg.positionz = positionz

        # Wait for Action Server to be ready
        self._movexyz_action_client.wait_for_server()
        
        # Get Future to a Goal Handle
        self._send_goal_future = self._movexyz_action_client.send_goal_async(goal_msg)

        # Callback for Goal Completion
        self._send_goal_future.add_done_callback(self.movexyz_goal_response_callback)

    # MoveXYZ Action Client Response
    def movexyz_goal_response_callback(self, future):
        
        # Get Result Early
        goal_handle = future.result()
        
        # If Send Goal Fail
        if not goal_handle.accepted:
            self.get_logger().info('Goal Rejected')
            return
        
        # If Send Goal Success
        self.get_logger().info('Goal Accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.movexyz_get_result_callback)
        
    #  MoveXYZ Action Client Feedback
    def movexyz_get_result_callback(self, future):
        
        # Initialise Result Variable
        result = future.result().result
        
        # Print Result(s)
        self.get_logger().info('Result: {0}'.format(result.result))
        
    # MoveYPR Action Client Goal
    def moveypr_send_goal(self, yaw, pitch, roll):

        # Debug
        self.get_logger().info('Sending Goal...')

        # Initial Action Variables
        goal_msg = MoveYPR.Goal()
        goal_msg.yaw    = yaw
        goal_msg.pitch  = pitch
        goal_msg.roll   = roll

        # Wait for Action Server to be ready
        self._moveypr_action_client.wait_for_server()
        
        # Get Future to a Goal Handle
        self._send_goal_future = self._moveypr_action_client.send_goal_async(goal_msg)

        # Callback for Goal Completion
        self._send_goal_future.add_done_callback(self.moveypr_goal_response_callback)

    # MoveYPR Action Client Response
    def moveypr_goal_response_callback(self, future):
        
        # Get Result Early
        goal_handle = future.result()
        
        # If Send Goal Fail
        if not goal_handle.accepted:
            self.get_logger().info('Goal Rejected')
            return
        
        # If Send Goal Success
        self.get_logger().info('Goal Accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.moveypr_get_result_callback)
        
    #  MoveXYZ Action Client Feedback
    def moveypr_get_result_callback(self, future):
        
        # Initialise Result Variable
        result = future.result().result
        
        # Print Result(s)
        self.get_logger().info('Result: {0}'.format(result.result))
        
    # MoveXYZ Coordinate Table
    _armXYZ = [
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
    
    # MoveYPR Coordinate Table
    _armYPR = [
        [ 0.0, 0.0, 090.0 ], # Neutral
        [ 0.0, 0.0, 150.0 ], # Drop
        [ 0.0, 0.0, 030.0 ], # Catch
    ]

# Main
def main(args=None):
    rclpy.init(args=args)

    bss_commander_node = BSSCommanderNode()

    rclpy.spin(bss_commander_node)
    
    # Shut Down Nav2
    bss_commander_node._navigator_action_client.lifecycleShutdown()
    
    # Call Node Destructor
    bss_commander_node.destroy_node()
    
    rclpy.shutdown


if __name__ == '__main__':
    main()
    