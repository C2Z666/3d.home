"""Map inspection only: no navigation controller, SDK, or movement publishers."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def setup(context):
    root=Path(__file__).resolve().parents[1]
    floor=LaunchConfiguration('floor').perform(context)
    if floor not in ('lower','upper'):
        raise ValueError('floor must be lower or upper')
    sim=ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)
    return [
        Node(package='nav2_map_server',executable='map_server',name='map_server',
             parameters=[{'use_sim_time':sim,'yaml_filename':str(root/'maps'/f'{floor}.yaml'),'frame_id':'map','topic_name':'map'}]),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_map_preview',
             parameters=[{'use_sim_time':sim,'autostart':True,'node_names':['map_server']}])]

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('floor',default_value='lower'),
        DeclareLaunchArgument('use_sim_time',default_value='false'),OpaqueFunction(function=setup)])
