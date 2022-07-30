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
        DeclareLaunchArgument(
            'arena_height', default_value='500',
            description='Arena height'
        ),
        DeclareLaunchArgument(
            'arena_width', default_value='500',
            description='Arena width'
        ),
        DeclareLaunchArgument(
            'agent_diameter', default_value= '50',
            description='Agent physical diameter'
        ),
        DeclareLaunchArgument(
            'mapf_solver', default_value= 'CBSSolver',
            description='MAPF Algorithm Class'
        ),
        DeclareLaunchArgument(
            'mapf_input', default_value= 'CBSInput',
            description='MAPF Algorithm Input Class'
        ),
        DeclareLaunchArgument(
            'goal_assigner', default_value= 'SimpleGoalAssigner',
            description='Goal Assigning Algorithm Class'
        ),
        DeclareLaunchArgument(
            'time_limit', default_value='0.0',
            description='MAPF Solving time limit'
        ),
        Node(
            package='arch_components',
            executable='planner',
            name='planner',
            parameters=[
                {
                    'tf_tag_arena': LaunchConfiguration('tf_tag_arena'),
                    'arena_height': LaunchConfiguration('arena_height'),
                    'arena_width': LaunchConfiguration('arena_width'),
                    'agent_diameter': LaunchConfiguration('agent_diameter'),
                    'mapf_solver': LaunchConfiguration('mapf_solver'),
                    'mapf_input': LaunchConfiguration('mapf_input'),
                    'goal_assigner': LaunchConfiguration('goal_assigner'),
                    'time_limit': LaunchConfiguration('time_limit'),
                }
            ]
        ),
    ])
