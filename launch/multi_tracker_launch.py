import os
import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration



def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='tracker_360',
            executable='theta_node',
            name='theta_node',
            output='screen'
        ),

        launch_ros.actions.Node(
            package='tracker_360',
            executable='multi_person_tracker.py',
            name='multi_person_tracker',
            output='screen',
            parameters=[],
            arguments=[os.path.join(os.getenv('ROS_WS', '/home/rcasal/ros2_ws'), 'install/tracker_360/lib/tracker_360/multi_person_tracker.py')]
        )


    ])