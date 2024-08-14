from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    host_ip = DeclareLaunchArgument
    
    front_scanner = Node(
        package="sick_safetyscanners2",
        executable="sick_safetyscanners2_node",
        name="sick_safetyscanners2_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "frame_id": "scan",
                "sensor_ip": "192.168.1.101",
                "host_ip": "192.168.1.194",
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
    )

    rear_scanner = Node(
        package="sick_safetyscanners2",
        executable="sick_safetyscanners2_node",
        name="sick_safetyscanners2_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "frame_id": "scan",
                "sensor_ip": "192.168.1.100",
                "host_ip": "192.168.1.194",
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
    )

    ### TODO: Add laser scan merger
    
    ld = LaunchDescription()
    ld.add_action(front_scanner)
    ld.add_action(rear_scanner)
    # ld.add_action(laser_scan_merger)
     
    return ld
