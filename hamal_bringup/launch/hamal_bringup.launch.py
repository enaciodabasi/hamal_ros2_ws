import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

from enum import Enum, auto
import os

class AutoName(Enum):
  @staticmethod
  def _generate_next_value_(name: str, start: int, count: int, last_values: list[str]) -> str:
      return name
  
class RobotMode(AutoName):
  AUTONOMOUS = auto(),
  MAPPING = auto(),
  HYBRID = auto(),
  MANUAL = auto()

def generate_launch_description():

    robot_operation_mode_arg = DeclareLaunchArgument(
       name="operation_mode",
       default_value="AUTONOMOUS"
    )
    
    use_slam = LaunchConfiguration("use_slam")
    
    declare_use_slam = DeclareLaunchArgument(
      'use_slam', default_value='false' 
    )

    ### Get robot description
    robot_description_content = Command(
        [
          PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("hamal_description"),
                              "urdf",
                              "hamal.urdf.xacro"
        ])
        ]
    )
    robot_description = {'robot_description': robot_description_content}
    ### Launch robot_state_publisher
    ## Run Condition: Always

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    ### Launch controllers and hardware
    ## Run Condition: Always
    control_launch_path = os.path.join(get_package_share_directory("hamal_control"), "launch", "hamal_control.launch.py")
    
    control_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=[control_launch_path]
        ),  
        #launch_arguments={'robot_description', robot_description}
    )
    scanner_launch_path = os.path.join(get_package_share_directory("hamal_mapping"), "launch", "hamal_laser_scanners.launch.py")
    laser_scanner_launch = IncludeLaunchDescription(
      launch_description_source=PythonLaunchDescriptionSource(
        launch_file_path=[scanner_launch_path]
      )
    )
    
    nav2_launch_path = os.path.join(get_package_share_directory("hamal_navigation"), "launch", "hamal_navigation.launch.py")
    nav2_launch = IncludeLaunchDescription(
      launch_description_source=PythonLaunchDescriptionSource(
        launch_file_path=[nav2_launch_path]
      )
    )
    
    #mapping_launch_path = os.path.join(get_package_share_directory("hamal_mapping"), "launch", "hamal_mapping.launch")
    #mapping_launch = IncludeLaunchDescription(
    #    launch_description_source=PythonLaunchDescriptionSource(
    #        launch_file_path=[control_launch_path]
    #    ),
    #    condition=IfCondition(use_slam)
    #)
    
    ### Get requested robot mode:
    ##robot_mode_str: str = LaunchConfiguration("operation_mode").perform()
    ##robot_mode_str = robot_mode_str.upper()
    ##robot_mode_opt = RobotMode._member_map_.get(robot_mode_str)
    ##robot_mode = RobotMode.AUTONOMOUS
    ##if robot_mode_opt is None:
    ##  pass
    ##else:
    ##  robot_mode = robot_mode_opt
    
    
    
    ld = LaunchDescription()
    ld.add_action(robot_state_pub_node)
    ld.add_action(control_launch)
    ld.add_action(laser_scanner_launch)
    #ld.add_action(nav2_launch)
    #ld.add_action(mapping_launch)

    #if robot_mode is RobotMode.AUTONOMOUS or robot_mode is RobotMode.HYBRID:
    #   ### Launch nav2
#
    #   ### Check if mode is hybrid:
    #   if robot_mode is RobotMode.HYBRID:  
    #    ## Launch hybrid navigation packages
    #    pass
    #   pass
    #elif robot_mode is RobotMode.MAPPING:
    #   ### Launch the mapping package
    #   pass
    #   
    #elif robot_mode is RobotMode.MANUAL:
    #   ### Only launch the manual driving programms
    #   pass
    
    

    return ld