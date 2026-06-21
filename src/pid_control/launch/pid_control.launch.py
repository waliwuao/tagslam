# -----------------------------------------------------------------------------
# PID Control launch file
# -----------------------------------------------------------------------------
import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    config_dir = LaunchConfig('config_dir')
    node = Node(
        package='pid_control',
        executable='pid_node.py',
        name='pid_control_node',
        output='screen',
        parameters=[{
            'config_file': [
                config_dir, '/pid_control.yaml'
            ],
            'tagslam_config': [
                config_dir, '/tagslam.yaml'
            ],
            'use_sim_time': LaunchConfig('use_sim_time'),
        }],
    )
    return [node]


def generate_launch_description():
    return launch.LaunchDescription([
        LaunchArg('config_dir',
                  default_value=[os.getcwd() + '/config'],
                  description='path to config directory'),
        LaunchArg('use_sim_time',
                  default_value=['False'],
                  description='whether to use sim time'),
        OpaqueFunction(function=launch_setup),
    ])
