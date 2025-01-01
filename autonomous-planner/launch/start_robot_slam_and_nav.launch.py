from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    test_robot_description_share = FindPackageShare(package='autonomous-planner').find('autonomous-planner')
    slam_params_file_path = os.path.join(test_robot_description_share, 'params/mapper_params_online_sync.yaml')
    params_file_path = os.path.join(test_robot_description_share, 'params/nav2_params.yaml')
    # Paths to the launch files
    robot_urdf_launch_dir = os.path.join(
        FindPackageShare('robot_urdf').find('robot_urdf'), 'launch')
    slam_toolbox_launch_dir = os.path.join(
        FindPackageShare('slam_toolbox').find('slam_toolbox'), 'launch')
    nav2_bringup_launch_dir = os.path.join(
        FindPackageShare('nav2_bringup').find('nav2_bringup'), 'launch')

    # Include the robot_urdf launch file
    robot_urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(robot_urdf_launch_dir, 'gazebo2.launch.py'))
    )
    # Include the slam_toolbox launch file with parameters
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_toolbox_launch_dir, 'online_sync_launch.py')),
        launch_arguments={'slam_params_file': slam_params_file_path}.items()
    )

    # Include the nav2_bringup launch file with parameters
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_launch_dir, 'navigation_launch.py')),
        launch_arguments={'params_file': params_file_path}.items()
    )

    return LaunchDescription([
        robot_urdf_launch,
        slam_toolbox_launch,
        nav2_bringup_launch
    ])