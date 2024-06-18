from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the YAML file
    yaml_file_path = '/home/jkang/vo_ws/mhe_vio/src/mhe_pogox_exp/config/parameters_realtime.yaml'
    print(yaml_file_path)
    return LaunchDescription([
        # Set use_sim_time to true
        Node(
            package='mhe_pogox_exp',
            executable='sub',
            name='pogox_Sub',
            output='screen',
            # prefix=['valgrind --tool=callgrind --dump-instr=yes -v --instr-atstart=yes'],
            parameters=[{'use_sim_time': False }, yaml_file_path]
        ),
    ])