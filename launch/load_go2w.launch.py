from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, Shutdown, SetLaunchConfiguration, \
     IncludeLaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, \
     TextSubstitution
from launch.conditions import LaunchConfigurationEquals, IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

PACKAGE_NAME = 'go2w_description'
def generate_launch_description():

     pkg_share = get_package_share_directory('go2w_description')
     urdf_file_path = os.path.join(pkg_share, 'urdf', 'go2w_description.urdf')
     with open(urdf_file_path, 'r') as infp:
          robot_desc = infp.read()

     return LaunchDescription([
          
          DeclareLaunchArgument(name='use_jsp', default_value='gui',
                                choices=['gui', 'jsp', 'none'],
                                description='Choose if joint_state_publisher is launched'),
          DeclareLaunchArgument(name='use_rviz', default_value='true',
                                choices=['true', 'false'],
                                description='Choose if rviz is launched'),
          DeclareLaunchArgument(name='namespace', default_value='',
                                description=
                                   'Choose a namespace for the launched topics.'),

          Node(package='joint_state_publisher',
               executable='joint_state_publisher',
               namespace=LaunchConfiguration('namespace'),
               condition=LaunchConfigurationEquals('use_jsp', 'jsp')),

          Node(package='joint_state_publisher_gui',
               executable='joint_state_publisher_gui',
               namespace=LaunchConfiguration('namespace'),
               condition=LaunchConfigurationEquals('use_jsp', 'gui')),

          Node(package='robot_state_publisher',
               executable='robot_state_publisher',
               parameters=[{'robot_description': ParameterValue(robot_desc, value_type=str)}],
               namespace=LaunchConfiguration('namespace')
          ),

          Node(package='rviz2',
               executable='rviz2',
               name='rviz2',
               arguments=[
                    "-d",
                    PathJoinSubstitution([FindPackageShare(PACKAGE_NAME), "config", "default.rviz"]),
               ],
          )
    ])