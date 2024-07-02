from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare 
from ament_index_python.packages import get_package_share_path
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    # Get the path to the YAML file
    estimation_yaml_file_name = "config/parameters_go1.yaml"
    estimation_yaml_file = os.path.join(get_package_share_path("go1_example"), estimation_yaml_file_name)
    
    vo_package_path = get_package_share_path("orbslam3")
    print(vo_package_path)
    
    decentral_node = Node(
        package='go1_example',
        executable='sub',
        name='est_sub',
        output='screen',
        prefix = "nice -n -19 taskset -c 1",
        parameters=[{'use_sim_time': False }, estimation_yaml_file]
    )
        
    vocab_arg = DeclareLaunchArgument(
        'vocabulary_path',
        default_value= os.path.join(vo_package_path, "vocabulary/ORBvoc.txt"),
        description='Path to vocabulary file.'
    )
    settings_arg = DeclareLaunchArgument(
        'settings_path',
        default_value= os.path.join(vo_package_path, "config/RealSense_D455_640_480.yaml"),
        description='Path to vocabulary file.'
    )
    rectify_arg = DeclareLaunchArgument(
        'do_rectify',
        default_value='false',
        description='Whether to rectify images before processing.'
    )            
    orbslam_node = Node(
        package='orbslam3',
        executable='stereo-decentralized',
        name='vo_sub',
        # output='screen',
        prefix = "nice -n -18 taskset -c 4,5,6,7,8,9,10",
        arguments=[
            LaunchConfiguration('vocabulary_path'),
            LaunchConfiguration('settings_path'),
            LaunchConfiguration('do_rectify')
        ],
        parameters=[{'use_sim_time': False }, estimation_yaml_file]
    )
    
    orien_node = Node(
        package='orien_est',
        executable='sub',
        name='orien_sub',
        # output='screen',
        prefix = "nice -n -19 taskset -c 11",
        parameters=[{'use_sim_time': False }, estimation_yaml_file]
    )
    
    # Create the launch description and populate
    ld = LaunchDescription([
        decentral_node,
        vocab_arg,
        settings_arg,
        rectify_arg,
        orbslam_node
    ])
    return ld