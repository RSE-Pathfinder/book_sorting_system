#! /usr/bin/env python3

# Time library
import time

# ROS2 Python
import rclpy
from rclpy.node import Node                                         # ROS2 Node Object
from rclpy.duration import Duration                                 # ROS2 Duration Object
from rclpy.action import ActionClient                               # ROS2 Action Object
from rclpy.executors import SingleThreadedExecutor                  # ROS2 Executor Object
from rclpy.timer import Timer                                       # ROS2 Timer Object
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup    # ROS2 Callback Group Object

# Communication with User Interface
from bss_controller_interface.srv import BSSControl 

# Communication with Robot Arm
from ros2_data.action import MoveXYZ    # Control Arm Position    (Coordinate)
from ros2_data.action import MoveYPR    # Control Arm Orientation (Coordinate)
from ros2_data.action import MoveROT    # Control Arm Orientation (Relative)
from ros2_data.action import MoveXYZW   # Control Arm Position & Orientation (Coordinate)

# Communication with Mobile Shelf
from geometry_msgs.msg import PoseStamped
from .submodules.robot_navigator import BasicNavigator, NavigationResult

SCOOP_STATE_NEUTRAL =   0
SCOOP_STATE_CATCH   =   1
SCOOP_STATE_DROOP   =   2

# MoveXYZ Action Client Node
class MoveXYZActionClient(Node):
    # Constructor
    def __init__(self, callback_group):
        
        # Construct Node
        super().__init__('movexyz_action_client')
        
        # Construct Action Client
        self._action_client = ActionClient(
            self, 
            MoveXYZ, 
            'MoveXYZ',
            callback_group=callback_group
            )
        
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

# MoveYPR Action Client Node
class MoveYPRActionClient(Node):
    # Constructor
    def __init__(self, callback_group):
        
        # Construct Node
        super().__init__('moveypr_action_client')
        
        # Construct Action Client
        self._action_client = ActionClient(
            self, 
            MoveYPR, 
            'MoveYPR',
            callback_group=callback_group
            )
        
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
    
# MoveROT Action Client Node
class MoveROTActionClient(Node):
    # Constructor
    def __init__(self, callback_group):
        
        # Construct Node
        super().__init__('moverot_action_client')
        
        # Construct Action Client
        self._action_client = ActionClient(
            self, 
            MoveROT, 
            'MoveROT',
            callback_group=callback_group
            )
        
        # Debug Statement
        self.get_logger().info('MoveROT Action Client Node Ready')

    # MoveXYZ Action Client Send Goal
    def send_goal(self, yaw, pitch, roll):
        
        # Debug Statement
        self.get_logger().info('Sending MoveROT Goal...')
        
        # Initialise Action Goal
        goal_msg = MoveROT.Goal()
        goal_msg.yaw    = yaw
        goal_msg.pitch  = pitch
        goal_msg.roll   = roll
        
        # Wait for Action Server
        self._action_client.wait_for_server()
        
        return self._action_client.send_goal_async(goal_msg)
    
# MoveXYZW Action Client Node
class MoveXYZWActionClient(Node):
    # Constructor
    def __init__(self, callback_group):
        
        # Construct Node
        super().__init__('movexyzw_action_client')
        
        # Construct Action Client
        self._action_client = ActionClient(
            self, 
            MoveXYZW, 
            'MoveXYZW',
            callback_group=callback_group
            )
        
        # Debug Statement
        self.get_logger().info('MoveXYZW Action Client Node Ready')

    # MoveXYZ Action Client Send Goal
    def send_goal(self, positionx, positiony, positionz, yaw, pitch, roll):
        
        # Initialise Action Goal
        goal_msg = MoveXYZW.Goal()
        goal_msg.positionx  = positionx
        goal_msg.positiony  = positiony
        goal_msg.positionz  = positionz
        goal_msg.yaw        = yaw
        goal_msg.pitch      = pitch
        goal_msg.roll       = roll
        
        # Debug Statement
        self.get_logger().info('Wait for MoveXYZW Action Server')
        
        # Wait for Action Server
        self._action_client.wait_for_server()
        
        # Send Goal & Check Future
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    # MoveXYZW Action Client Response Callback
    def goal_response_callback(self, future):
        
        # Get Goal Handler for Result
        goal_handle = future.result()
        
        # Return if Send Goal Rejected
        if not goal_handle.accepted:
            self.get_logger().info('MoveXYZW Goal Rejected')
            return
        
        self.get_logger().info('MoveXYZW Goal Accepted')

        # Get Result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        
    #  MoveXYZW Action Client Result Callback
    def get_result_callback(self, future):
        
        # Initialise Result Variable
        result = future.result().result
        
        # Print Result(s)
        self.get_logger().info('MoveYPR Result: {0}'.format(result.result)) 

# BSS Commander Node
class BSSCommanderNode(Node):
    # Constructor
    def __init__(self):
        
        # Create Node
        super().__init__('bss_commander_node')
        
        commander_callback_group = MutuallyExclusiveCallbackGroup()
        
        # User Interface BSSControl Action Server Constructor
        self._bsscontrol_service_server = self.create_service(
            BSSControl,                                     # ROS Action
            'bss_ui_command',                               # ROS Topic
            self.bss_ui_command_callback,                   # ROS Action Server Callback
            )
        
        # Mobile Shelf Navigator Action Client Constructor
        self._navigator_action_client = BasicNavigator()
        
        # Robot Arm MoveXYZ Action Client Node Constructor
        self._movexyz_action_client = MoveXYZActionClient(commander_callback_group)
        
        # Robot Arm MoveYPR Action Client Node Constructor
        self._moveypr_action_client = MoveYPRActionClient(commander_callback_group)
        
        # Robot Arm MoveROT Action Client Node Constructor
        self._moverot_action_client = MoveROTActionClient(commander_callback_group)
        
        # Robot Arm MoveXYZW Action Client Node Constructor
        self._movexyzw_action_client = MoveXYZWActionClient(commander_callback_group)
        
        # # Initialize Navigator Goal Pose
        # _navigator_goal_pose = PoseStamped()
        # _navigator_goal_pose.header.frame_id = 'map'
        # _navigator_goal_pose.header.stamp = self._navigator_action_client.get_clock().now().to_msg()
        # _navigator_goal_pose.pose.position.x = 0.0
        # _navigator_goal_pose.pose.position.y = 0.0
        # _navigator_goal_pose.pose.position.z = 0.0
        # _navigator_goal_pose.pose.orientation.x = 0.0
        # _navigator_goal_pose.pose.orientation.y = 0.0
        # _navigator_goal_pose.pose.orientation.z = 0.0
        # _navigator_goal_pose.pose.orientation.w = 0.0
        # self._navigator_goal_pose_table.append(_navigator_goal_pose)
        # self.get_logger().info(self._navigator_goal_pose_table[0].header.frame_id)
        
        # Initialize MoveXYZ, MoveYPR Variables
        self._index = 0
        self._row = 0
        self._col = 0
        self._scoopstate = SCOOP_STATE_CATCH
        
        # Initialize MoveXYZW Action Client
        self._movexyzw_action_client.send_goal(
            self._movexyz_goal_pose_table[self._index][self._row][self._col][self._scoopstate][0], # x-coordinate
            self._movexyz_goal_pose_table[self._index][self._row][self._col][self._scoopstate][1], # y-coordinate
            self._movexyz_goal_pose_table[self._index][self._row][self._col][self._scoopstate][2], # z-coordinate
            self._moveypr_goal_pose_table[self._index][self._scoopstate][0], # yaw
            self._moveypr_goal_pose_table[self._index][self._scoopstate][1], # pitch
            self._moveypr_goal_pose_table[self._index][self._scoopstate][2]  # roll
            )
        
        self.get_logger().info('BSS Arm Action Node Ready')

    # BSSControl Service Callback
    def bss_ui_command_callback(self, request, response):
        
        # Debug
        self.get_logger().info('BSS UI Server Request...')
        
        # Initialise Action Variables
        self._index      =   request.index
        self._row        =   request.row
        self._col        =   request.col
        self._scoopstate =   request.scoop_state
        
        try:
            self.moveXYZW_send_goal()
        
            # Define BSSControl Response
            response.result = "REQUEST SUCCESS"
        
        except:
            # Define BSSControl Response
            response.result = "REQUEST FAILURE"
            
        return response
    
    def moveNavigator_send_goal(self):
        
        # Call Navigator Action Client
        self._navigator_action_client.waitUntilNav2Active()
        
        # Send PoseStamped Object
        goal_pose = PoseStamped()
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
                    #self._navigator_action_client.goThroughPoses([goal_pose_alt])
            
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
    
    # MoveXYZW Send Goal
    def moveXYZW_send_goal(self):
        
        self._movexyzw_action_client.send_goal(
            self._movexyz_goal_pose_table[self._index][self._row][self._col][self._scoopstate][0], # x-coordinate
            self._movexyz_goal_pose_table[self._index][self._row][self._col][self._scoopstate][1], # y-coordinate
            self._movexyz_goal_pose_table[self._index][self._row][self._col][self._scoopstate][2], # z-coordinate
            self._moveypr_goal_pose_table[self._index][self._scoopstate][0], # yaw
            self._moveypr_goal_pose_table[self._index][self._scoopstate][1], # pitch
            self._moveypr_goal_pose_table[self._index][self._scoopstate][2]  # roll
            )

    # Navigator Coordinate Table
    _navigator_goal_pose_table = [
        # Queue Position 1
        [
            [ 0.0, 0.0, 0.0 ],
            [ 0.0, 0.0, 0.0, 0.0 ]
        ],
        # Parking Lot 1
        [
            [ 0.0, 0.0, 0.0 ],
            [ 0.0, 0.0, 0.0, 0.0 ]
        ]
    ]

    # MoveXYZ Coordinate Table
    _movexyz_goal_pose_table = [
        # Counter
        [ 
            [
                [
                    [ 0.8, 0.0, 0.6 ],  # Neutral
                    [ 0.8, 0.0, 0.4 ],  # Catch
                    [ 0.8, 0.0, 0.8 ]   # Droop
                ],  # Index 0, Row 0, Col 0
                [
                    [ 0.8, 0.0, 1.2 ],  # Neutral
                    [ 0.8, 0.0, 1.0 ],  # Catch
                    [ 0.8, 0.0, 1.4 ]   # Droop
                ]   # Index 0, Row 0, Col 1
            ]
        ],
        # Shelf 1
        [  
            [
                [
                    [ 0.4, 0.8, 1.0 ],  # Neutral
                    [ 0.4, 0.8, 0.8 ],  # Catch
                    [ 0.4, 0.8, 1.2 ]   # Droop
                ],  # Index 1, Row 0, Col 0
                [
                    [ 0.4, 0.8, 1.6 ],  # Neutral
                    [ 0.4, 0.8, 1.4 ],  # Catch
                    [ 0.4, 0.8, 1.8 ]   # Droop
                ]   # Index 1, Row 0, Col 1
            ],
            [
                [
                    [ -0.4, 0.8, 1.0 ],  # Neutral
                    [ -0.4, 0.8, 0.8 ],  # Catch
                    [ -0.4, 0.8, 1.2 ]   # Droop
                ],  # Index 1, Row 1, Col 0
                [
                    [ -0.4, 0.8, 1.6 ],  # Neutral
                    [ -0.4, 0.8, 1.4 ],  # Catch
                    [ -0.4, 0.8, 1.8 ]   # Droop
                ]   # Index 1, Row 1, Col 1
            ]
        ]
    ]

    # MoveYPR Coordinate Table
    _moveypr_goal_pose_table = [
        # Counter
        [
            [ 090.0, 000.0, 090.0 ],    # Neutral
            [ 090.0, 000.0, 060.0 ],    # Catch
            [ 090.0, 000.0, 120.0 ]     # Droop
        ],
        # Shelf 1
        [
            [ 180.0, 000.0, 090.0 ],    # Neutral
            [ 180.0, 000.0, 060.0 ],    # Catch
            [ 180.0, 000.0, 120.0 ]     # Droop
        ]
    ]

# Main
def main(args=None):
    rclpy.init(args=args)

    # Node Constructor
    bss_commander_node = BSSCommanderNode()
    executor = SingleThreadedExecutor()
    executor.add_node(bss_commander_node)
    
    try:
        bss_commander_node.get_logger().info('Nodes Ready, BSS Commander is running. Shut down with CTRL-C')
        executor.spin()
        
    except KeyboardInterrupt:
        bss_commander_node.get_logger().info('Keyboard Interrupt, shutting down.')
    
    # Shut Down Nav2
    bss_commander_node._navigator_action_client.lifecycleShutdown()
    
    # Node Destructor
    bss_commander_node.destroy_node()
    
    # # Shutdown ROS2
    # rclpy.shutdown

if __name__ == '__main__':
    main()
    