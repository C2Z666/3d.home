from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def setup(context):
    root=Path(__file__).resolve().parents[1]
    site=yaml.safe_load((root/'config/site.yaml').read_text())
    if not site.get('maps_validated') or not site.get('scan_source_validated'):
        raise RuntimeError('config/site.yaml: validate maps and current-floor scan projection before Nav2 startup')
    floor=LaunchConfiguration('floor').perform(context)
    if floor not in ('lower','upper'): raise ValueError('floor must be lower or upper')
    cfg=str(root/'config/nav2.yaml'); use_sim=ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)
    common=[cfg,{'use_sim_time':use_sim}]
    nodes=[Node(package='nav2_map_server',executable='map_server',name='map_server',parameters=[cfg,{'use_sim_time':use_sim,'yaml_filename':str(root/'maps'/f'{floor}.yaml')}]),
      Node(package='nav2_planner',executable='planner_server',name='planner_server',parameters=common),
      Node(package='nav2_controller',executable='controller_server',name='controller_server',parameters=common,remappings=[('cmd_vel','/d1/nav/cmd_vel_raw')]),
      Node(package='nav2_bt_navigator',executable='bt_navigator',name='bt_navigator',parameters=[cfg,{'use_sim_time':use_sim,'default_nav_to_pose_bt_xml':str(root/'config/navigate.xml'),'default_nav_through_poses_bt_xml':str(root/'config/navigate_through.xml')}]),
      Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_d1',parameters=[{'use_sim_time':use_sim,'autostart':True,'node_names':['map_server','planner_server','controller_server','bt_navigator'],'bond_timeout':4.}])]
    return nodes

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('floor',default_value='lower'),DeclareLaunchArgument('use_sim_time',default_value='false'),OpaqueFunction(function=setup)])
