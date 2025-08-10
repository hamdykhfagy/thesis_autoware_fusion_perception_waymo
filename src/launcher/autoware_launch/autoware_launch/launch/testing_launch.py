from launch import LaunchDescription
from launch_ros.actions import Node
from tracetools_launch.action import Trace

def generate_launch_description():
    return LaunchDescription([
        Trace(session_name='test_trace'),
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            output='screen'
        ),
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            output='screen'
        ),
    ])
