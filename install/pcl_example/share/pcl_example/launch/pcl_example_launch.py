from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pcl_example',
            executable='pcl_example_node',
            name='pcl_example_node',
            output='screen',
            parameters=[]
        )
    ])
