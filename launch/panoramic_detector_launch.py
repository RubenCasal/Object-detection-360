import os
import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration



def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='panoramic_object_detector',
            executable='theta_node',
            name='theta_node',
            output='screen'
        ),

        launch_ros.actions.Node(
            package='panoramic_object_detector',
            executable='panoramic_detector.py',
            name='panoramic_detector',
            output='screen',
            parameters=[],
            arguments=[os.path.join(os.getenv('ROS_WS', '/home/rcasal/ros2_ws'), 'install/panoramic_object_detector/lib/panoramic_object_detector/panoramic_detector.py')]
        )


    ])