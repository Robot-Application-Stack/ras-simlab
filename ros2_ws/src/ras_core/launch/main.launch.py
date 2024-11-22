import os
import yaml
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_context import LaunchContext
from launch.logging import get_logger
from ament_index_python.packages import get_package_share_directory
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path

def generate_launch_description():

    # Load the robot configuration
    moveit_config = (
        MoveItConfigsBuilder(
            context=LaunchContext(),
            controllers_name="fake_controllers",
            robot_type="lite",
            dof=6,
            hw_ns="lite",
            prefix="",
            ros2_control_plugin="ign_ros2_control/IgnitionSystem",
            kinematics_params_filename="lite6_default_kinematics.yaml"
        )
        .robot_description()
        .trajectory_execution(file_path="config/lite6/fake_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
        arguments=["--log-level", "debug"],
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("xarm_moveit_config") + "/rviz/moveit.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Publish TF
    robot_state_publisher_arm = Node(
        name='robot_state_publisher',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
    )

    # Node to spawn joint trajectory controller
    spawn_controllers_manipulator = Node(
        package="controller_manager", 
        executable="spawner",
        name="spawner_mani",
        arguments=['joint_trajectory_controller'],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    # Node to spawn joint state broadcaster
    spawn_controllers_state = Node(
        package="controller_manager", 
        executable="spawner",
        arguments=['joint_state_broadcaster'],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )



    launch_world = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('ros_ign_gazebo'),
            'launch', 'ign_gazebo.launch.py')]),
    launch_arguments=[('gz_args', [' -r /ras_sim_lab/ros2_ws/src/ras_sim/worlds/lab.sdf'])]
    )

    # gazebo spawn entity node
    gazebo_spawn_entity_node = Node(
        package="ros_gz_sim",
        executable="create",
        output='screen',
        arguments=[
            '-string', moveit_config.robot_description['robot_description'],
            '-name', 'UF_ROBOT',
            '-x', '-1.239280',
            '-y', '-3.298410',
            '-z', '1.120070',
            '-Y', '-1.57'
        ],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription(
        [
            rviz_node,
            launch_world,
            gazebo_spawn_entity_node,
            robot_state_publisher_arm,
            run_move_group_node,
            spawn_controllers_manipulator,
            spawn_controllers_state
        ]
    )