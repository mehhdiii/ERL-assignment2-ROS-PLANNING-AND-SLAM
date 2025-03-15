from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import yaml

def generate_launch_description():

    test_robot_description_share = FindPackageShare(package='autonomous_planner').find('autonomous_planner')
    built_map_path = os.path.join(test_robot_description_share, 'configs/mymap')
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
        PythonLaunchDescriptionSource(os.path.join(robot_urdf_launch_dir, 'rosbot.launch.py'))
    )

        # Path for the temporary YAML file
    temp_slam_params_path = os.path.join(
        test_robot_description_share, 'params/temp.yaml'
    )

    # Create the temporary YAML file with updated parameters
    with open(slam_params_file_path, 'r') as file:
        params = yaml.safe_load(file)

    params['slam_toolbox']['ros__parameters']['map_file_name'] = built_map_path

    try:
        with open(temp_slam_params_path, 'w') as file:
            yaml.dump(params, file)
    except Exception as e:
        print('Error writing the temporary YAML file: ', e)


    # Include the slam_toolbox launch file with parameters
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_toolbox_launch_dir, 'online_sync_launch.py')),
        launch_arguments={'slam_params_file': temp_slam_params_path}.items()
    )

    # Include the nav2_bringup launch file with parameters
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_launch_dir, 'navigation_launch.py')),
        launch_arguments={'params_file': params_file_path}.items()
    )

    # Plansys2 launch file
    plansys2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            FindPackageShare('plansys2_bringup').find('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py'
        )),
        launch_arguments={'model_file': os.path.join(test_robot_description_share, 'pddl/rosbot.pddl')}.items()
    )

# aruco
    aruco_node = Node(
        package='ros2_aruco',
        executable='aruco_node',
        name='aruco_node',
        output='screen'
    )

    move_action_node = Node(
        package='autonomous_planner',
        executable='move_action_node',
        name='move_action_node',
        output='screen',
        parameters=[]
    )
    
    scan_action_node = Node(
        package='autonomous_planner',
        executable='scan_marker_action_node',
        name='scan_marker_action_node',
        output='screen',
        parameters=[]
    )
    
    

    return LaunchDescription([
        robot_urdf_launch,
        slam_toolbox_launch,
        nav2_bringup_launch,
        plansys2_launch,
        move_action_node,
        scan_action_node,
        aruco_node
    ])
