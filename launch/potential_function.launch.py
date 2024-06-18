import os
from launch import LaunchDescription
from launch_ros.actions import SetParameter
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='potential_function',
            executable='potential_function',
            name='potential_function_node_0',
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory('potential_function'),
                'config', 'config.yaml'), 
                os.path.join(get_package_share_directory('potential_function'),
                'config', 'topics.yaml')]
        ),
        Node(
            package='potential_function',
            executable='potential_function',
            name='potential_function_node_1',
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory('potential_function'),
                'config', 'config.yaml'), 
                os.path.join(get_package_share_directory('potential_function'),
                'config', 'topics.yaml')]
        ),
        Node(
            package='potential_function',
            executable='potential_function',
            name='potential_function_node_2',
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory('potential_function'),
                'config', 'config.yaml'), 
                os.path.join(get_package_share_directory('potential_function'),
                'config', 'topics.yaml')]
        ),
        Node(
            package='potential_function',
            executable='potential_function',
            name='potential_function_node_3',
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory('potential_function'),
                'config', 'config.yaml'), 
                os.path.join(get_package_share_directory('potential_function'),
                'config', 'topics.yaml')]
        ),
        Node(
            package='potential_function',
            executable='potential_function',
            name='potential_function_node_4',
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory('potential_function'),
                'config', 'config.yaml'), 
                os.path.join(get_package_share_directory('potential_function'),
                'config', 'topics.yaml')]
        ),
        Node(
            package='potential_function',
            executable='target_pose_list_publisher.py',
            name='target_pose_list_publisher_node',
            output='screen',
            parameters=[
                os.path.join(
                    get_package_share_directory('potential_function'),
                    'config', 'target_pose_list.yaml'
                )]
        )
    ])