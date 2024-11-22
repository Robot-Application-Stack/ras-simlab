from pathlib import Path
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    lab_pkg = Path(get_package_share_directory('ras_asset_lab_lab1'))
    gazebo_pkg = Path(get_package_share_directory('ros_ign_gazebo'))
    world_path = lab_pkg/'worlds'/'lab.sdf'
    launch_world = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        [str(gazebo_pkg/'launch'/'ign_gazebo.launch.py')]),
    launch_arguments=[('gz_args', [f' -r {str(world_path)}'])]
    )

    return LaunchDescription([
        launch_world
    ])