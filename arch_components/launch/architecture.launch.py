from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'tf_tag_arena', default_value='mocap',
            description='Arena target frame'
        ),
        Node(
            package='arch_components',
            executable='planner',
            name='planner',
            parameters=[
                {
                    'tf_tag_arena': LaunchConfiguration('tf_tag_arena'),
                }
            ]
        ),
    ])
