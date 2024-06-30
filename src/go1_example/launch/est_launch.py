from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the YAML file
    yaml_file_path = '/home/jkang/vo_ws/mhe_vio/src/go1_exp/config/parameters_go1.yaml'
    print(yaml_file_path)
    return LaunchDescription([
        # Set use_sim_time to true
        Node(
            package='go1_exp',
            executable='sub',
            name='est_sub',
            output='screen',
            prefix = "nice -n -19 taskset -c 1",
            parameters=[{'use_sim_time': False }, yaml_file_path]
        ),
    ])