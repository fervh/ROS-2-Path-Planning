import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os.path

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'csv_file',
            default_value='map2.csv',
            description='Name of the CSV file to read'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='bfs',
            description='Planning algorithm to use: bfs, greedy or astar'
        ),
        DeclareLaunchArgument(
            'sleep_time', 
            default_value='0.1', 
            description='Time delay for time.sleep'
        ),
        Node(
            package='planification',
            executable='readcsv',
            name='readcsv',
            parameters=[{'csv_file': LaunchConfiguration('csv_file')}],
        ),

        Node(
            package='planification',
            executable='occupancygrid',
            name='occupancygrid',
        ),

        Node(
            package='planification',
            executable='startend',
            name='startend',
        ),

        Node(
            package='planification',
            executable=LaunchConfiguration('mode'),
            name='planner_node',
            parameters=[{'sleep_time': LaunchConfiguration('sleep_time')}],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d' + os.path.join(get_package_share_directory('planification'), 'config', 'rviz_config.rviz')]
        ),
    ])
