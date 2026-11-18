"""Existing Navfn planner in a separate namespace, without any controller."""
from pathlib import Path
import sys
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(root/'scripts'))
    from scene_registry import scene_bundle
    from site_identity import verify
    name = LaunchConfiguration('scene').perform(context)
    scene = scene_bundle(root, name)
    from floor_routes import load_registry,read_state
    registry=load_registry(root,scene);state=read_state(root,registry)
    floor=registry['floors'][state.get('active_nav_floor') or state.get('actual_floor') or registry['initial_floor']]['map_key']
    if name == 'old': verify(root)
    config = yaml.safe_load((root/'config/nav2.yaml').read_text())
    config['global_costmap']['global_costmap']['ros__parameters']['static_layer']['map_topic'] = '/d1/preview/map'
    # Preserve the existing planner/costmap settings. Use the requested final yaw.
    config['planner_server']['ros__parameters']['GridBased']['use_final_approach_orientation'] = False
    config['planner_server']['ros__parameters']['GridBased']['tolerance'] = 0.0
    params = root/'work/planner_preview_params.yaml'
    params.parent.mkdir(exist_ok=True)
    params.write_text(yaml.safe_dump({'d1': {'preview': config}}))
    ns = 'd1/preview'
    return [
        Node(package='nav2_map_server', executable='map_server', name='map_server', namespace=ns,
             parameters=[str(params), {'yaml_filename':str(scene['maps'][floor]['yaml'])}]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', namespace=ns,
             parameters=[str(params)], remappings=[('tf','/tf'), ('tf_static','/tf_static')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager', namespace=ns,
             parameters=[{'autostart':True, 'node_names':['map_server','planner_server'], 'bond_timeout':4.0}])]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('scene', default_value='old'), OpaqueFunction(function=setup)])
