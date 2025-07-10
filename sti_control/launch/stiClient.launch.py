import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # -- get param in src package
    config = os.path.join(os.getcwd(), "src", "launch_pkg", "config", "param.yaml")
    
    return LaunchDescription([
        Node(
            package='sti_control',
            executable='stiClient_socketio',
            name='stiClient',
            output='screen',
            emulate_tty=True
            # parameters=[config]
        )
    ])