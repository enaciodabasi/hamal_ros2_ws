from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():

    #host_ip = DeclareLaunchArgument
    
    front_scanner = Node(
        package="sick_safetyscanners2",
        executable="sick_safetyscanners2_node",
        name="sick_safetyscanners2_node_front",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "frame_id": "front_scan_link",
                "sensor_ip": "192.168.1.101",
                "host_ip": "192.168.1.70",
                "interface_ip": "0.0.0.0",
                "host_udp_port": 0,
                "channel": 0,
                "channel_enabled": True,
                "skip": 2,
                "angle_start": -2.1,
                "angle_end": 1.2,
                "time_offset": 0.0,
                "general_system_state": True,
                "derived_settings": True,
                "measurement_data": True,
                "intrusion_data": True,
                "application_io_data": True,
                "use_persistent_config": False,
                "min_intensities": 0.0,
            }
        ],
        remappings=[
            ('/scan', '/front_scan'),
        ]
    )

    rear_scanner = Node(
        package="sick_safetyscanners2",
        executable="sick_safetyscanners2_node",
        name="sick_safetyscanners2_node_rear",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "frame_id": "rear_scan_link",
                "sensor_ip": "192.168.1.100",
                "host_ip": "192.168.1.70",
                "interface_ip": "0.0.0.0",
                "host_udp_port": 0,
                "channel": 0,
                "channel_enabled": True,
                "skip": 2,
                "angle_start": -2.2,
                "angle_end": 1.45,
                "time_offset": 0.0,
                "general_system_state": True,
                "derived_settings": True,
                "measurement_data": True,
                "intrusion_data": True,
                "application_io_data": True,
                "use_persistent_config": False,
                "min_intensities": 0.0,
            }
        ],
        remappings=[
            ('/scan', '/rear_scan'),
        ]
    )
    
    #merger_launch_path = os.path.join(get_package_share_directory("ira_laser_tools"), "launch", "laserscan_multi_merger.launch")
    #
    #merger_launch = IncludeLaunchDescription(
    #    launch_description_source=PythonLaunchDescriptionSource(
    #        launch_file_path=[merger_launch_path]
    #    ),
    #    #launch_arguments={'robot_description', robot_description}
    #)
    
    merger_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ira_laser_tools"),
                "launch/laserscan_multi_merger.launch",
            )
        )
    )
    
    ld = LaunchDescription()
    ld.add_action(front_scanner)
    ld.add_action(rear_scanner)
    ld.add_action(merger_launch)
    
    return ld
