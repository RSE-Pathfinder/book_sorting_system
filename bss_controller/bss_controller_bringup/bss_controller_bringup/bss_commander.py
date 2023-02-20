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
from ros2_data.action import MoveXYZ    # Control Arm Position
from ros2_data.action import MoveYPR    # Control Arm Orientation

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
        self._movexyz_action_client = ActionClient(
            self, 
            MoveXYZ,    # ROS2 Action
            'MoveXYZ'   # ROS2 Topic
        )
        
        # Debug Statement
        self.get_logger().info('MoveXYZ Action Client Node Ready')

    # MoveXYZ Action Client Goal
    def movexyz_send_goal(self, positionx, positiony, positionz):
        
        # Debug Statement
        self.get_logger().info('Sending MoveXYZ Goal...')
        
        # Initialise Action Goal
        goal_msg = MoveXYZ.Goal()
        goal_msg.positionx = positionx
        goal_msg.positiony = positiony
        goal_msg.positionz = positionz
        
        # Wait for Action Server
        self._movexyz_action_client.wait_for_server()
        
        # Send Goal
        return self._movexyz_action_client.send_goal_async(goal_msg)

# MoveYPR Action Client Node
class MoveYPRActionClient(Node):
    # Constructor
    def __init__(self):
        
        # Construct Node
        super().__init__('moveypr_action_client')
        
        # Construct Action Client
        self._moveypr_action_client = ActionClient(
            self, 
            MoveYPR,    # ROS2 Action
            'MoveYPR'   # ROS2 Topic
        )
        
        # Debug Statement
        self.get_logger().info('MoveYPR Action Client Node Ready')

    # MoveYPR Action Client Goal
    def moveypr_send_goal(self, yaw, pitch, roll):
        
        # Debug Statement
        self.get_logger().info('Sending MoveYPR Goal...')
        
        # Initialise Action Goal
        goal_msg = MoveYPR.Goal()
        goal_msg.yaw    = yaw
        goal_msg.pitch  = pitch
        goal_msg.roll   = roll
        
        # Wait for Action Server
        self._moveypr_action_client.wait_for_server()
        
        # Send Goal
        return self._moveypr_action_client.send_goal_async(goal_msg)
    
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
        
        # # Robot Arm MoveXYZ Action Client Constructor
        # self._movexyz_action_client = MoveXYZActionClient()
        
        # # Robot Arm MoveYPR Action Client Constructor
        # self._moveypr_action_client = MoveYPRActionClient()
        
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
        
        # # Create PoseStamped Object
        # goal_poses = []
        # goal_pose = PoseStamped()
        # goal_pose.header.frame_id = 'map'
        # goal_pose.header.stamp = self._navigator_action_client.get_clock().now().to_msg()
        # goal_pose.pose.position.x = 0.0
        # goal_pose.pose.position.y = 0.0
        # goal_pose.pose.position.z = 0.0
        # goal_pose.pose.orientation.x = 0.0
        # goal_pose.pose.orientation.y = 0.0
        # goal_pose.pose.orientation.z = 0.0
        # goal_pose.pose.orientation.w = 0.0
        # goal_poses.append(goal_pose)
        
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
        
        # Robot Arm MoveXYZ Action Client Constructor
        _movexyz_action_client = MoveXYZActionClient()
        
        # Call MoveXYZ Action Client
        futureXYZ = _movexyz_action_client.movexyz_send_goal(
            self._armXYZ[index][row][col][0], # x-coordinate
            self._armXYZ[index][row][col][1], # y-coordinate
            self._armXYZ[index][row][col][2], # z-coordinate
            )
        while futureXYZ.result() is not None:
            rclpy.spin_until_future_complete(_movexyz_action_client, futureXYZ)
            
        _movexyz_action_client.destroy_node()
        
        # Robot Arm MoveYPR Action Client Constructor
        _moveypr_action_client = MoveYPRActionClient()
        
        # Call MoveYPR Action Client
        futureYPR = _moveypr_action_client.moveypr_send_goal(
            self._armYPR[scoop_state][0],        # yaw
            self._armYPR[scoop_state][1],        # pitch
            self._armYPR[scoop_state][2],        # roll
            )
        while futureYPR.result() is not None:
            rclpy.spin_until_future_complete(_moveypr_action_client, futureYPR)
            
        _moveypr_action_client.destroy_node()
        
        # Indicate Goal Success
        goal_handle.succeed()
        
        # Define MoveArm Action Result
        result = MoveArm.Result()
        result.result = feedback.feedback
        return result
        
    # MoveXYZ Coordinate Table
    _armXYZ = [
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
    _armYPR = [
        [ 180.0, 0.0, 090.0 ],    # Scoop Neutral
        [ 180.0, 0.0, 150.0 ],    # Scoop Droop
        [ 180.0, 0.0, 030.0 ]     # Scoop Catch
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
    