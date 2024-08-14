import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
import launch
def generate_launch_description():

    robot_description_contents = LaunchConfiguration("robot_description")
    namespace = LaunchConfiguration("ns")

    robot_description_arg = DeclareLaunchArgument(
            name="robot_description",
            default_value=""
        )    

    control_config = PathJoinSubstitution(
        [
            FindPackageShare("hamal_control"),
            "config",
            "hamal_control.yaml"
        ]
    )

    hardware_interface_config = PathJoinSubstitution(
        [
            FindPackageShare("hamal_hardware"),
            "config",
            "hamal_hardware.yaml"
        ]
    )

    if launch.conditions.IfCondition(robot_description_contents):
      return launch.actions.EmitEvent(event=launch.events.Shutdown(
                reason="Could not get robot description."
            ))
    
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description_contents, control_config, hardware_interface_config],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["midi_base_controller", "--controller-manager", "/controller_manager"],
    )

    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    ld = LaunchDescription()
    
    ld.add_action(robot_description_arg)
    ld.add_action(controller_manager_node)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(robot_controller_spawner)
    ld.add_action(delay_robot_controller_spawner_after_joint_state_broadcaster_spawner)


    return ld