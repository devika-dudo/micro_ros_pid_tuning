from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'pid_motor_tune'
    
    config_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'pid_tune_params.yaml'
    )
    
    signal_gen_node = Node(
        package=package_name,
        executable='signal_generator',
        name='signal_generator',
        output='screen',
        parameters=[config_file]
    )
    
    pid_param_node = Node(
        package=package_name,
        executable='pid_param_publisher',
        name='pid_param_publisher',
        output='screen',
        parameters=[config_file]
    )
    
    plotjuggler_node = ExecuteProcess(
        cmd=['ros2', 'run', 'plotjuggler', 'plotjuggler'],
        output='screen'
    )
    
    rqt_reconfigure_node = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_reconfigure', 'rqt_reconfigure'],
        output='screen'
    )
    
    return LaunchDescription([
        signal_gen_node,
        pid_param_node,
        plotjuggler_node,
        rqt_reconfigure_node,
    ])
