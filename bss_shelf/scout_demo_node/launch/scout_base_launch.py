from launch import LaunchDescription

from launch_ros.actions import Node

def generate_launch_description():

    scout_node = Node(
                    package='scout_demo_node',
                    executable='scout_node',
                    name='base',
                    namespace='scout')
    
    ld = LaunchDescription()
    ld.add_action(scout_node)

    return ld