
"""
Launches rviz with the go1 urdf file.
"""

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
    # 패키지 경로 찾기
     pkg_share = get_package_share_directory('go2w_description')
    
    # URDF 파일 경로 설정
     urdf_file_path = os.path.join(pkg_share, 'urdf', 'go2w_description.urdf')
     with open(urdf_file_path, 'r') as infp:
          robot_desc = infp.read()

     return LaunchDescription([
          
          DeclareLaunchArgument(name='use_jsp', default_value='jsp',
                                choices=['gui', 'jsp', 'none'],
                                description='Choose if joint_state_publisher is launched'),
          DeclareLaunchArgument(name='use_rviz', default_value='true',
                                choices=['true', 'false'],
                                description='Choose if rviz is launched'),
        #   DeclareLaunchArgument(name='use_nav2_links', default_value='false',
        #                         choices=['true', 'false'],
        #                         description='Use Nav2 frames in URDF'),
        #   DeclareLaunchArgument(name='fixed_frame', default_value='base',
        #                         description='Fixed frame for RVIZ'),
          DeclareLaunchArgument(name='namespace', default_value='',
                                description='Choose a namespace for the launched topics.'),

          SetLaunchConfiguration(name='config_file',
                                 value='default.rviz'),
          # SetLaunchConfiguration(name='model',
          #                        value=PathJoinSubstitution([FindPackageShare('go2w_description'),
          #                                                    'urdf',
          #                                                    'robot.xacro'])),
          # SetLaunchConfiguration(name='rvizconfig',
          #                        value=PathJoinSubstitution([FindPackageShare('go2w_description'),
          #                                                    'config',
          #                                                    LaunchConfiguration('config_file')])),

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