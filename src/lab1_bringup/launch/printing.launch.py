from launch import LaunchDescription 
from launch.actions import DeclareLaunchArgument, ExecuteProcess 
from launch_ros.actions import Node 
from launch.substitutions import LaunchConfiguration 
from launch.conditions import IfCondition, UnlessCondition 

def generate_launch_description():
    use_terminals = LaunchConfiguration('use_terminals')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_terminals',
            default_value='0',
            description='Launch each node in a separate terminal (1=yes, 0=no)'
        ),

        ExecuteProcess(
            condition=IfCondition(use_terminals),
            cmd=['gnome-terminal', '--','bash', 'ros2', 'run', 'task4', 'service'], #'-c', 'source /opt/ros/humble/setup.bash;',
            #name='service_terminal',
            output='screen'
        ),

        Node(
            condition=UnlessCondition(use_terminals),
            package='task4',
            executable='service',
            output='screen'
        ),

        ExecuteProcess(
            condition=IfCondition(use_terminals),
            cmd=['gnome-terminal', '--', 'bash', 'ros2', 'run', 'task4', 'client', '--ros-args' '-p', 'x1:=1.0', '-p', 'y1:=1.0', '-p', 'x2:=2.0', '-p', 'y2:=2.0',], #Copilot used to generate the parameters values
            #name='client_terminal',
            output='screen'
        ),

        Node(
            condition=UnlessCondition(use_terminals),
            package='task4',
            executable='client',
            output='screen',
            parameters = [{'x1': 1.0,
                'y1': 1.0,
                'x2': 2.0,
                'y2': 2.0}]
        )
    ])